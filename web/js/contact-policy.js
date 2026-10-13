// Runtime contact policy shared by the UI and regression tests.
// Planned gripper-pad contact with the selected cube is expected while picking,
// carrying, releasing and withdrawing. Other arm contacts still stop playback.

export function contactNeedsArmStop(key, { plannedPick = null, plannedPhase = '', holdingId = null } = {}) {
  if (/^.+ cube × (board|desk)$/i.test(key) || /^.+ cube × .+ cube$/i.test(key)) return false;
  const match = key.match(/^(?:(left|right) pad|gripper(?:Front|Back)) × (.+) cube$/i);
  if (!match) return true;
  return !(match.at(-1) === plannedPick
    && (/^(Descend|Grip|Lift|Lower|Release|Withdraw)/i.test(plannedPhase) || holdingId === match.at(-1)));
}
