const REQUIRED_DIMENSIONS = [
  'board', 'boardCenter', 'shoulderHeight', 'upperArm', 'forearm', 'wrist',
  'gripperOpenLength', 'gripperClosedLength', 'gripperOpenGap',
  'gripperClosedGap', 'testCube',
];

export function validateArmSpec(spec) {
  const errors = [];
  if (spec?.metadata?.units !== 'millimetres') errors.push('metadata.units must be "millimetres"');
  if (spec?.metadata?.schemaVersion !== 2) errors.push('metadata.schemaVersion must be 2');
  for (const key of REQUIRED_DIMENSIONS) {
    if (spec?.dimensions?.[key] == null) errors.push(`dimensions.${key} is required`);
  }
  if (!Array.isArray(spec?.joints) || spec.joints.length !== 6) errors.push('exactly six joints are required');
  for (const joint of spec?.joints ?? []) {
    if (!/^S[1-6]$/.test(joint.id)) errors.push(`invalid joint id ${joint.id}`);
    if (![joint.min, joint.max, joint.home, joint.park].every(Number.isFinite)) errors.push(`${joint.id} count fields must be numbers`);
    if (joint.min > joint.max || joint.home < joint.min || joint.home > joint.max || joint.park < joint.min || joint.park > joint.max) {
      errors.push(`${joint.id} reference counts must be inside its historical bounds`);
    }
    if (joint.boundsStatus !== 'historical-unvalidated') errors.push(`${joint.id} bounds must retain historical-unvalidated provenance`);
  }
  if (spec?.fixture?.upperPlate?.top !== 25) errors.push('fixture upper surface must remain at the photo-derived 25 mm height');
  if (errors.length) throw new Error(`Invalid arm specification:\n- ${errors.join('\n- ')}`);
  return spec;
}

export async function loadArmSpec(url = '/arm-spec.json', fetchImpl = fetch) {
  const response = await fetchImpl(url);
  if (!response.ok) throw new Error(`Could not load arm specification (${response.status})`);
  return validateArmSpec(await response.json());
}

function metres(value) {
  if (Array.isArray(value)) return value.map(metres);
  if (typeof value === 'number') return value / 1000;
  if (value && typeof value === 'object') return Object.fromEntries(Object.entries(value).map(([key, item]) => [key, metres(item)]));
  return value;
}

export function createRenderSpec(spec) {
  return {
    ...spec,
    dimensions: metres(spec.dimensions),
    fixture: metres(spec.fixture),
    world: {
      ...spec.world,
      cubes: spec.world.cubes.map((cube) => ({ ...cube, position: metres(cube.position) })),
      dropZones: Object.fromEntries(Object.entries(spec.world.dropZones).map(([key, position]) => [key, metres(position)])),
    },
  };
}
