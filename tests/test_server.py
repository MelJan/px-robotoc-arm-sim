"""The sandbox server end to end: robot-arm's HTTP API, the camera routes and the text interface."""
import io
import json
import tempfile
import time
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import urlopen

import numpy as np
import pytest

from armsim.cli import Shell
from armsim.client import ArmError, HttpArmClient
from armsim.robotarm import HOME, PARK
from armsim.server import Sandbox
from armsim.tasks import HttpWorld, PickPlace


@pytest.fixture(scope='module')
def served():
    with tempfile.TemporaryDirectory() as directory:
        sandbox = Sandbox(artifacts=Path(directory))
        server = sandbox.start(port=0)
        base = f'http://127.0.0.1:{server.server_address[1]}'
        yield sandbox, base
        sandbox.stop()


def test_state_and_power_on_through_http(served):
    sandbox, base = served
    client = HttpArmClient(base)
    state = client.state()
    assert state['mode'] == 'preview' and not state['armed']
    client.power_on()
    state = client.wait_idle()
    assert state['armed'] and {int(k): v for k, v in state['commands'].items()} == HOME
    for channel in range(6):
        assert sandbox.physics.joint_angle(channel) == pytest.approx(sandbox.physics.calibration.angle(channel, HOME[channel]), abs=1.5)


def test_sandbox_routes_and_move_pose(served):
    sandbox, base = served
    client = HttpArmClient(base)
    blocks = client.sim('blocks')
    assert set(blocks) == {'red', 'green', 'blue', 'yellow', 'roof'}
    assert client.sim('state')['held'] is None
    client.settings(demo_speed=100)
    target = dict(HOME)
    target[1] += 30
    client.move_pose(target)
    assert client.commands() == target
    bad = dict(target)
    bad[5] = 700
    with pytest.raises(ArmError):
        client.move_pose(bad)
    with urlopen(f'{base}/sim/snapshot.png?camera=top', timeout=10) as response:
        assert response.read().startswith(b'\x89PNG')
    with pytest.raises(HTTPError):
        urlopen(f'{base}/sim/snapshot.png?camera=nope', timeout=10)
    assert client.sim('reset', {})['ok']


def test_camera_routes_serve_rendered_frames_like_the_pi(served):
    sandbox, base = served
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        with urlopen(f'{base}/api/camera/status', timeout=5) as response:
            status = json.load(response)
        if status.get('connected') and status.get('latest'):
            break
        time.sleep(0.2)
    assert status['enabled'] and status['source'] == 'mujoco'
    with urlopen(f'{base}/api/camera/snapshot.jpg', timeout=10) as response:
        jpeg = response.read()
        metadata = json.loads(response.headers['X-Camera-Frame'])
    assert jpeg.startswith(b'\xff\xd8') and metadata['width'] == 1280 and metadata['sequence'] >= 1
    with urlopen(f'{base}/api/camera/capture', timeout=10) as response:
        capture = json.load(response)
    assert capture['depth_png'] is None and capture['rgb_jpeg']


def test_pick_and_place_over_http_with_the_text_interface(served):
    sandbox, base = served
    out = io.StringIO()
    shell = Shell(base, out=out)
    shell.client.settings(demo_speed=100)
    shell.run_line('home')
    shell.run_line('blocks')
    shell.run_line('run red T1')
    shell.run_line('state')
    text = out.getvalue()
    assert 'red moved to T1' in text and 'red     x=' in text
    slot = sandbox.scene['slots']['T1']
    time.sleep(0.5)
    pos, _ = sandbox.physics.body_pose('red')
    assert np.linalg.norm(np.array(pos[:2]) - (slot['x'], slot['y'])) < 0.006
    assert shell.client.commands() == HOME
    log = Path(tempfile.mkdtemp()) / 'run.jsonl'
    shell.run_line(f'log {log}')
    assert log.exists() and 'poses written' in out.getvalue()
    shell.run_line('nonsense')
    assert 'unknown command' in out.getvalue()


def test_demo_import_and_playback_through_the_text_interface(served, tmp_path):
    sandbox, base = served
    record = dict(schema_version=1, recorded_at='2026-10-11T10:00:00+00:00', name='Wave',
                  positions=[dict(name='Up', counts={str(c): v for c, v in dict(HOME, **{}).items()}),
                             dict(name='Turn', counts={'0': 380, '1': 427, '2': 403, '3': 220, '4': 480, '5': 650})],
                  scope='commissioning', units='PCA9685_counts', nominal_frequency_hz=60,
                  validated_limits=False, position_feedback=False)
    source = tmp_path / 'wave.json'
    source.write_text(json.dumps(record))
    out = io.StringIO()
    shell = Shell(base, out=out)
    shell.demo_store = type(shell.demo_store)(sandbox.session.demo_store.directory.parent, 'synthetic', shell.demo_store.bounds)
    shell.run_line('speed 100')
    shell.run_line('home')
    shell.run_line(f'demo import {source}')
    shell.run_line('demo list')
    shell.run_line('demo load Wave')
    shell.run_line('demo run')
    text = out.getvalue()
    assert 'imported as' in text and 'Wave (imported from wave.json' in text and 'demo finished' in text
    assert shell.client.commands()[0] == 380
    assert shell.client.state()['demo']['status'] == 'complete'
