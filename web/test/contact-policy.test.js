import test from 'node:test';
import assert from 'node:assert/strict';
import { contactNeedsArmStop } from '../js/contact-policy.js';

test('planned pad contact remains allowed through release and withdrawal', () => {
  for (const phase of ['Descend 1/2', 'Grip red', 'Lift 2/2', 'Lower 2/2', 'Release', 'Withdraw 1/2']) {
    assert.equal(contactNeedsArmStop('left pad × red cube', { plannedPick: 'red', plannedPhase: phase }), false, phase);
  }
});

test('unexpected gripper and arm contacts still request a stop', () => {
  assert.equal(contactNeedsArmStop('right pad × blue cube', { plannedPick: 'red', plannedPhase: 'Release' }), true);
  assert.equal(contactNeedsArmStop('forearm × red cube', { plannedPick: 'red', plannedPhase: 'Release' }), true);
  assert.equal(contactNeedsArmStop('left pad × red cube', { plannedPick: 'red', plannedPhase: 'move S1' }), true);
  assert.equal(contactNeedsArmStop('left pad × red cube', { plannedPick: 'red', holdingId: 'red' }), false);
  assert.equal(contactNeedsArmStop('red cube × board'), false);
});

test('Rapier gripper contacts use the same planned-contact exception', () => {
  assert.equal(contactNeedsArmStop('gripperFront × red cube', {
    plannedPick: 'red', plannedPhase: 'Grip red', holdingId: null,
  }), false);
  assert.equal(contactNeedsArmStop('gripperBack × blue cube', {
    plannedPick: 'red', plannedPhase: 'HOME verified', holdingId: null,
  }), true);
});
