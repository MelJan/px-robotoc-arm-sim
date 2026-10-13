// Every number the simulator uses, in one place.
// Lengths in cm, angles in degrees, servo values in PCA9685 counts at 60 Hz.
// Frame (docs/arm-dimensions.md): origin on the board top under the S1 axis, Z up,
// +Y from the back edge toward the front, +X to the right seen from the robot.
//
// The schematic (docs/images/arm-dimensions.png) is canon: link lengths, gripper
// values, board and cube come from it unchanged. Sizes of servo bodies and brackets
// are only for drawing and collision, taken from the photos and the MG996R datasheet.

export const ARM = {
  H_S2: 9.6,    // board top to S2 axis
  L2: 10.5,     // S2 shoulder to S3 elbow
  L3: 12.6,     // S3 elbow to S4 wrist flexion
  L4: 6.7,      // S4 wrist flexion to gripper flange (S5)
  G_BLOCK: 11.0 // flange to the centre of a held cube
};

export const BOARD = { w: 30, d: 38, t: 1.0, axisToBackEdge: 1.5 };

// The arm's base: two U-beam brackets stacked into a box, its top plate BASE.h above the board. 3,3 cm from
// the photos (photos/park-gripper-on-base.jpg: the drawn 2,5 was visibly too low). The S1 shaft (BASE.s1Shaft)
// and the S2 axis (ARM.H_S2) keep their heights: the S1 servo sits deeper in the base.
export const BASE = { h: 3.3, s1Shaft: 6.42, offsetY: 2.2 };

export const CUBE = 2.65;

// Gripper parallel linkage, gripper-local u (sideways) and v (down from the flange), cm.
// Fitted in docs/arm-dimensions.md so that it reproduces the measured values exactly:
// open 10,0 / gap 4,5; holding the cube 11,0 / gap 2,65; closed 11,5 / gap 0.
export const GRIPPER = {
  gear: [0.85, 1.4],       // gear centre (u, v)
  outer: [2.2, 1.9],       // outer link pivot (u, v)
  link: 3.5,               // both links
  tip: [-1.515, 6.664],    // pad contact point relative to the gear-link end
  pad: { h: 1.2, t: 0.27, w: 1.0 },
  plate: { w: 6.4, h: 2.4, t: 0.4 },
  thetaOpen: 56.4,         // link angle from vertical at S6 = 490
  thetaClosed: 11.0,       // at S6 = 650
};

// Servo table. q_home is the model angle (0 = all links straight up,
// positive S2..S4 tilt toward +Y, positive S1 turns +Y toward +X) at the HOME count.
// dir: +1 if increasing counts increases the model angle.
//
// q_home and dir are NOT measured (HANDOVER.md, open item 1). The defaults below were
// fitted to the team's taught demo "step1-hacks-10-direct-approach": with them the
// "Single approach" pose puts the cube centre at 1,3 cm height, 19 cm out, gripper
// pointing down, and PARK rests the gripper on the board. The fitted directions
// match the controller's historical sign table (S2 -1, S3 +1, S4 -1).
// Change them in the Calibration panel once they are measured.
export const SERVOS = [
  { ch: 0, name: 'S1', joint: 'Base rotation',  home: 320, park: 330, low: 90,  high: 660, cpd: 260 / 85, qHome: 0,  dir: -1 },
  { ch: 1, name: 'S2', joint: 'Shoulder',       home: 427, park: 540, low: 185, high: 660, cpd: 220 / 90, qHome: 6,  dir: -1 },
  { ch: 2, name: 'S3', joint: 'Elbow',          home: 403, park: 580, low: 90,  high: 660, cpd: 240 / 90, qHome: 62, dir: +1 },
  { ch: 3, name: 'S4', joint: 'Wrist flexion',  home: 220, park: 195, low: 90,  high: 660, cpd: 250 / 90, qHome: 84, dir: -1 },
  // The live controller's HOME has S5 = 250 (docs/controller-live-2026-10-12.md).
  // At 480 (the documented HOME) the jaws close along the arm's sideways axis.
  { ch: 4, name: 'S5', joint: 'Wrist rotation', home: 250, park: 480, low: 90,  high: 660, cpd: 230 / 90, qHome: 90, dir: -1 },
  { ch: 5, name: 'S6', joint: 'Gripper',        home: 550, park: 650, low: 490, high: 660, cpd: null, open: 490, grip: 590, closed: 650 },
];

// Controller behaviour copied from lutich/robot-arm (control/domain/motion.py, docs/api.md).
export const CONTROLLER = {
  manualSpeed: 20,      // counts/s for move, jog, go_pose
  demoSpeed: 20,        // default demo playback speed, 1..100 counts/s
  demoSpeedMax: 100,
  stepSize: 10,         // jog step
  // Trajectory: cubic smoothstep, all joints share one duration, which is the
  // slowest joint's 1,5 x |delta| / speed (the smoothstep peaks at 1,5x average).
  peakFactor: 1.5,
};

// Printed ArUco mat (lutich/robot-arm config/camera-table-xy). Camera table frame:
// origin at the middle of the mat edge nearest the arm, +X away from the arm, +Y to the
// arm's left, mm. Board frame: x = -Y/10, y = X/10 + d.
export const MAT = {
  w: 28, d: 36,          // cm
  // d: S1 axis to the mat's near edge, cm. Read from the overhead photo
  // docs/images/photos/setup-top-view-base.jpg: the board's back edge lies at about mat
  // X = 10 mm (the paper overhangs it), and S1 is 1,5 in front of that edge, so S1 is at
  // about X = 25 mm. Not measured; HANDOVER.md open item 3.
  sToEdge: -2.5,
};

// OAK camera, fitted from the mat markers (docs/tracking-plan.md, section 2).
export const CAMERA = {
  posCamMm: [480.8, -8.3, 371.7], // camera table frame
  fPx: 952, w: 1280, h: 720,
};

// The two cameras that watch the mat (photos/setup-two-cameras-*.jpg). Poses are first guesses in the
// camera table frame (mm); each camera is calibrated from the mat markers in its own frames, and that
// calibration replaces the guess. Both come from the controller's API (GET only): the OAK at
// /api/camera/*, the webcam at /api/camera2/* (status, snapshot.jpg, stream.mjpg, capture).
export const CAMERAS = [
  { id: 'oak', name: 'OAK on the stand', api: '/api/camera', w: 1280, h: 720, fPx: 952, posMm: [480.8, -8.3, 371.7], lookAtMm: [175, 0, 0] },
  { id: 'webcam', name: 'Webcam on the monitor', api: '/api/camera2', w: 1280, h: 720, fPx: 900, posMm: [600, -320, 420], lookAtMm: [170, 0, 0] },
];

// AprilTags (36h11) stuck to the arm (photos/arm-tags-*.jpg), for tracking the real arm with the cameras.
// Tags on one rigid part move together, so they act as one object: a camera that sees only one of them
// still adds to the same part, and two cameras seeing different tags of a part still meet in one pose.
// Each tag: link = fk() frame it is fixed to; centre, outward normal and "up" (direction of the tag's top
// edge as printed) in that frame, cm; size = side of the black square (1,9 cm: measured by triangulating
// the corners of 107 and 109 from both cameras, 2026-10-13). Positions come from the photos (2 to 3 mm);
// "up" is only a first guess: the tracker finds each tag's real turn from its corners.
export const ARM_TAGS = [
  // Calibrated 2026-10-13 from 3 poses (2 live snapshot pairs + the close-up photos, S5 spread 120 degrees,
  // js/jointfit.js estimateJointAxis): the layouts that make both parts turn about one S5 axis. The S5 axis
  // came out 10,6 degrees and 0,5 cm from the model's, the circle fit 0,55 cm; the wrist+gripper pair fit
  // dropped from 17-50 px to 3-9 px. Open: how far along the S5 axis the gripper tags sit relative to the
  // flange is not observable from joint motion (here the gripper tags end 2,6 cm below the flange, which the
  // photos roughly confirm, but the gripper model puts its S6 servo higher). Measure: S5 hub to tag 107 centre.
  // wrist part (moves with S1..S4): S5 servo and its bracket
  { key: 'wrist-s5', id: 109, link: 'wrist', centre: [1.02, -0.73, 3.98], normal: [-0.01, -1, -0.06], up: [-0.98, -0.01, 0.17], size: 1.9, on: 'S5 servo, wide face' },
  { key: 'wrist-plate', id: 108, link: 'wrist', centre: [3.63, 0.65, 2.45], normal: [0.98, 0.01, -0.17], up: [0.17, -0.06, 0.98], size: 1.9, on: 'S5 bracket side plate, +X (horn side)' },
  // gripper part (moves with S1..S5): S6 servo
  { key: 'hand-top', id: 107, link: 'hand', centre: [-1.69, -0.99, 2.87], normal: [-0.09, -0.29, -0.95], up: [-0.99, 0, 0.11], size: 1.9, on: 'S6 servo, face toward the wrist' },
  { key: 'hand-end', id: 106, link: 'hand', centre: [0.86, -0.18, 3.57], normal: [0.99, 0, -0.11], up: [0.09, 0.29, 0.95], size: 1.9, on: 'S6 servo, cable end' },
];
/** The rigid parts that carry tags: what moves them and where their joint axis is (for the overlay). */
export const ARM_PARTS = {
  wrist: { name: 'wrist (S4 bracket + S5 servo)', joints: 'S1–S4', last: 'S4', colour: 0x8e5cf7, axis: ['elbow', [0, 0, ARM.L3]] },
  hand: { name: 'gripper (S6 servo + jaws)', joints: 'S1–S5', last: 'S5', colour: 0x19b8d0, axis: ['wrist', [0, 0, ARM.L4]] },
};

// Motion planning defaults (docs/pick-and-place-plan.md, docs/tracking-plan.md).
export const PLAN = {
  hover: 5.0,           // approach height above a cube on the mat
  hoverStacked: 2.0,    // above level 1 and higher
  descentStep: 1.0,     // cm between IK waypoints on a vertical descent
  clearance: 0.05,      // collision tolerance, cm (0,5 mm)
  transitMargin: 0.15,  // planned paths keep 1,5 mm between the arm (or a held cube) and other cubes
};

export const COLOURS = {
  orange: 0xe8792b, blue: 0x2f6fd6, yellow: 0xf2c230, green: 0x3aa356, red: 0xd23b3b,
};
