#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace webots;

const double L_HIP_PITCH_SIGN = 1.0;
const double R_HIP_PITCH_SIGN = 1.0;
const double L_KNEE_SIGN = 1.0;
const double R_KNEE_SIGN = 1.0;
const double L_ANKLE_PITCH_SIGN = 1.0;
const double R_ANKLE_PITCH_SIGN = 1.0;

const double L_HIP_ROLL_SIGN = 1.0;
const double R_HIP_ROLL_SIGN = 1.0;
const double L_ANKLE_ROLL_SIGN = 1.0;
const double R_ANKLE_ROLL_SIGN = 1.0;

const double L_SHOULDER_PITCH_SIGN = 1.0;
const double R_SHOULDER_PITCH_SIGN = 1.0;

const double ROBOT_LOCAL_FORWARD_X = -1.0;
const double HIP_FORWARD_DIR = 1.0;
const double ROLL_TO_LEFT_DIR = 1.0;

// The mesh/feet point toward local -X. Webots reports positions in the world
// frame, so debug projects motion onto that local forward axis instead of dx.
const double MOTOR_VELOCITY = 5.0;
const double MOTOR_TORQUE = 94.0;
const double ANKLE_MOTOR_VELOCITY = 8.0;
const double ANKLE_MOTOR_TORQUE = 80.0;

const double GAIT_CYCLE_TIME = 2.02;
const double STANCE_FRACTION = 0.72;
const double SWING_FRACTION = 1.0 - STANCE_FRACTION;
const double GAIT_START_PHASE = 0.5;
const double START_BLEND_TIME = 1.40;
const double COMMAND_FILTER_TIME = 0.22;
const double SHIFT_TIME = 0.20;
const double SWING_TIME = 0.31;
const double DOWN_TIME = 0.15;
const bool RESCUE_STAND_ONLY = false;
const bool DEBUG_WALK = false;

const double ARM_SWING = 0.36;
const double ARM_SWING_LAND = 0.30;
const double ARM_SWING_SHIFT = 0.14;
const double ARM_ROLL_SWING = 0.0;
const double ARM_ROLL_LAND = 0.0;
const double ARM_ROLL_SHIFT = 0.0;
const double ARM_ELBOW_BASE = 0.62;
const double ARM_ELBOW_BACK_GAIN = 0.08;
const double TORSO_YAW = 0.0;
const double TORSO_FORWARD_LEAN = -0.062;
const double TORSO_PITCH_SHIFT = -0.025;
const double TORSO_PITCH_WALK = -0.045;

const double HIP_ROLL_SHIFT = 0.018;
const double ANKLE_ROLL_SUPPORT = 0.004;
const double ANKLE_ROLL_PREP = 0.0;
const double SWING_HIP_ROLL_CENTER = 0.0;
const double SHIFT_SUPPORT_HIP_FORWARD = 0.08;
const double SHIFT_TRAILING_HIP_BACK = -0.14;
const double SUPPORT_HIP_BACK = -0.18;
const double SWING_HIP_FORWARD = 0.66;
const double LAND_HIP_BACK = -0.16;
const double LAND_HIP_FORWARD = 0.52;
const double SUPPORT_KNEE = 0.22;
const double SWING_PREP_KNEE = 0.24;
const double SWING_KNEE = 0.86;
const double LAND_KNEE = 0.10;
const double SUPPORT_ANKLE_PUSH = 0.03;
const double LAND_SUPPORT_ANKLE = 0.03;
const double SWING_ANKLE_LIFT = -0.14;
const double LAND_ANKLE = -0.02;

const double HIP_SWING_BACK = -0.18;
const double HIP_SWING_FORWARD = 0.66;
const double KNEE_LANDING_ABSORB = 0.05;
const double KNEE_STANCE_FLEX = 0.05;
const double ANKLE_TOE_UP = -0.15;
const double ANKLE_FLAT = -0.03;
const double ANKLE_PUSH_OFF = 0.10;

const double HEEL_STRIKE_HIP = 0.58;
const double MID_STANCE_HIP = 0.02;
const double TOE_OFF_HIP = -0.32;
const double EARLY_SWING_HIP = 0.20;
const double LATE_SWING_HIP = 0.74;
const double LOADING_KNEE = 0.30;
const double MID_STANCE_KNEE = 0.18;
const double TOE_OFF_KNEE = 0.50;
const double PEAK_SWING_KNEE = 1.02;
const double SWING_RECOVERY_KNEE = 0.30;
const double HEEL_STRIKE_ANKLE = 0.0;
const double FOOT_FLAT_ANKLE = 0.00;
const double MID_STANCE_ANKLE = 0.0;
const double TOE_OFF_ANKLE = 0.0;
const double SWING_TOE_UP_ANKLE = 0.40;
const double SWING_CLEARANCE_ANKLE = 0.32;
const double ANKLE_PITCH_LIMIT = 0.85;
const double IK_BLEND = 0.35;
const double IK_THIGH_LENGTH = 0.18;
const double IK_SHIN_LENGTH = 0.32;
const double IK_STANCE_ANKLE_DOWN = 0.47;
const double IK_SWING_FOOT_LIFT = 0.030;
const double IK_SWING_FOOT_PITCH = 0.09;
const double GROUND_FOOT_PITCH = 0.0;
const double SWING_FOOT_TOE_UP_PITCH = 0.03;
const double SWING_FOOT_CLEARANCE_PITCH = -0.26;
const double SWING_FOOT_LANDING_PITCH = -0.22;
const double SWING_ANKLE_DOWN_EARLY = -0.42;
const double SWING_ANKLE_DOWN_MID = -0.58;
const double SWING_ANKLE_DOWN_LANDING = -0.38;
const double IK_NEUTRAL_HIP = 0.02;
const double IK_NEUTRAL_KNEE = MID_STANCE_KNEE;
const double IK_NEUTRAL_ANKLE = 0.0;

const double DEFAULT_TURN_COMMAND = 0.0;
const double TURN_COMMAND_MAX = 0.0;
const double TURN_WAIST_YAW = 0.0;
const double TURN_NECK_YAW = 0.0;
const double TURN_STRIDE_ASYMMETRY = 0.0;
const double TURN_LEAN_ROLL = 0.0;
const double TURN_ANKLE_ROLL = 0.0;

const double LATERAL_BALANCE_ROLL_GAIN = 0.08;
const double LATERAL_BALANCE_RATE_GAIN = 0.004;
const double LATERAL_BALANCE_MAX = 0.006;
const double COM_TARGET_LATERAL_LIMIT = 0.045;
const double COM_BALANCE_GAIN = 0.16;
const double COM_BALANCE_DEADBAND = 0.008;
const double COM_BALANCE_MAX = 0.006;
const double COM_BALANCE_FILTER_TIME = 0.28;
const double HIP_ROLL_LIMIT = 0.028;
const double ANKLE_ROLL_LIMIT = 0.012;

struct MotorHandle {
  Motor *motor;
  PositionSensor *sensor;
};

typedef std::map<std::string, double> PoseMap;

std::map<std::string, MotorHandle> motors;
PoseMap lastAppliedPose;
GPS *leftFootGps = nullptr;
GPS *rightFootGps = nullptr;
InertialUnit *bodyImu = nullptr;
double turnCommand = DEFAULT_TURN_COMMAND;

double clamp(double x, double lo, double hi) {
  if (x < lo)
    return lo;
  if (x > hi)
    return hi;
  return x;
}

double smoothStep(double x) {
  x = clamp(x, 0.0, 1.0);
  return x * x * x * (x * (x * 6.0 - 15.0) + 10.0);
}

double lerp(double a, double b, double u) {
  return a + (b - a) * u;
}

double cubicHermite(double p0, double m0, double p1, double m1, double u) {
  u = clamp(u, 0.0, 1.0);
  const double u2 = u * u;
  const double u3 = u2 * u;
  return (2.0 * u3 - 3.0 * u2 + 1.0) * p0 +
         (u3 - 2.0 * u2 + u) * m0 +
         (-2.0 * u3 + 3.0 * u2) * p1 +
         (u3 - u2) * m1;
}

double cosineEase(double x) {
  x = clamp(x, 0.0, 1.0);
  return 0.5 - 0.5 * std::cos(M_PI * x);
}

double rollingEase(double x) {
  x = clamp(x, 0.0, 1.0);
  return lerp(x, cosineEase(x), 0.45);
}

double softBell(double x) {
  return std::sin(M_PI * smoothStep(x));
}

double wrap01(double x) {
  x = std::fmod(x, 1.0);
  if (x < 0.0)
    x += 1.0;
  return x;
}

double leftPhaseForWalkTime(double walkTime) {
  return wrap01(walkTime / GAIT_CYCLE_TIME + GAIT_START_PHASE);
}

double rightPhaseForWalkTime(double walkTime) {
  return wrap01(leftPhaseForWalkTime(walkTime) + 0.5);
}

double rampUp(double x) {
  return smoothStep(clamp(x, 0.0, 1.0));
}

double rampDown(double x) {
  return 1.0 - rampUp(x);
}

double segmentStep(double x, double start, double end) {
  return smoothStep((x - start) / (end - start));
}

double capPositive(double desired, double limit) {
  if (limit <= 0.0)
    return desired;
  return desired < limit ? desired : limit;
}

void setMotor(const std::string &name, double value, double lo, double hi) {
  std::map<std::string, MotorHandle>::iterator it = motors.find(name);
  if (it == motors.end() || it->second.motor == nullptr)
    return;

  it->second.motor->setPosition(clamp(value, lo, hi));
}

void registerMotor(Robot *robot, int timestep, const std::string &name) {
  MotorHandle handle;
  handle.motor = robot->getMotor(name);
  handle.sensor = nullptr;

  if (handle.motor) {
    std::cout << "found motor: " << name << std::endl;
    const bool ankleMotor = name.find("ankle") != std::string::npos;
    const double desiredVelocity = ankleMotor ? ANKLE_MOTOR_VELOCITY : MOTOR_VELOCITY;
    const double desiredTorque = ankleMotor ? ANKLE_MOTOR_TORQUE : MOTOR_TORQUE;
    handle.motor->setVelocity(capPositive(desiredVelocity, handle.motor->getMaxVelocity()));
    handle.motor->setAvailableTorque(capPositive(desiredTorque, handle.motor->getMaxTorque()));

    const std::string sensorName = name + "_sensor";
    handle.sensor = robot->getPositionSensor(sensorName);
    if (handle.sensor)
      handle.sensor->enable(timestep);
  } else {
    std::cout << "WARNING missing motor: " << name << std::endl;
  }

  motors[name] = handle;
}

double poseValue(const PoseMap &p, const std::string &name) {
  std::map<std::string, double>::const_iterator it = p.find(name);
  if (it == p.end())
    return 0.0;
  return it->second;
}

PoseMap mixPose(const PoseMap &a, const PoseMap &b, double u) {
  u = smoothStep(u);

  PoseMap out = a;

  for (std::map<std::string, double>::const_iterator it = b.begin(); it != b.end(); ++it) {
    double av = poseValue(a, it->first);
    double bv = it->second;
    out[it->first] = lerp(av, bv, u);
  }

  return out;
}

PoseMap filterPose(const PoseMap &previous, const PoseMap &target, double alpha) {
  alpha = clamp(alpha, 0.0, 1.0);
  if (previous.empty())
    return target;

  PoseMap out = target;

  for (std::map<std::string, double>::const_iterator it = target.begin(); it != target.end(); ++it) {
    const double av = poseValue(previous, it->first);
    out[it->first] = lerp(av, it->second, alpha);
  }

  return out;
}

void setArmSwing(PoseMap &p, double leftArmPitch, double shoulderRoll, double elbowGain) {
  double effort = clamp(std::fabs(leftArmPitch) / ARM_SWING, 0.0, 1.0);
  double forwardElbow = clamp(ARM_ELBOW_BASE - elbowGain * effort, 0.18, 2.2);
  double backwardElbow = clamp(ARM_ELBOW_BASE + ARM_ELBOW_BACK_GAIN * effort, 0.0, 2.2);

  p["left_shoulder_pitch"] = leftArmPitch;
  p["right_shoulder_pitch"] = -leftArmPitch;
  p["left_shoulder_roll"] = shoulderRoll;
  p["right_shoulder_roll"] = -shoulderRoll;

  p["left_elbow_pitch"] = leftArmPitch > 0.0 ? forwardElbow : backwardElbow;
  p["right_elbow_pitch"] = leftArmPitch < 0.0 ? forwardElbow : backwardElbow;
}

void setTorso(PoseMap &p, double yaw, double pitch) {
  p["waist_yaw"] = yaw;
  p["waist_pitch"] = pitch;
}

void applyTurnToPose(PoseMap &p) {
  double turn = clamp(turnCommand, -TURN_COMMAND_MAX, TURN_COMMAND_MAX);
  if (std::fabs(turn) < 0.001)
    return;

  p["waist_yaw"] += TURN_WAIST_YAW * turn;
  p["neck_yaw"] += TURN_NECK_YAW * turn;

  double lean = TURN_LEAN_ROLL * turn * ROLL_TO_LEFT_DIR;
  p["left_hip_roll"] += lean;
  p["right_hip_roll"] += lean;
  p["left_ankle_roll"] -= TURN_ANKLE_ROLL * turn * ROLL_TO_LEFT_DIR;
  p["right_ankle_roll"] -= TURN_ANKLE_ROLL * turn * ROLL_TO_LEFT_DIR;

  // Positive turnCommand means curve left: the right/outside leg takes the
  // longer step while the left/inside leg shortens slightly.
  p["left_hip_pitch"] -= TURN_STRIDE_ASYMMETRY * turn * HIP_FORWARD_DIR;
  p["right_hip_pitch"] += TURN_STRIDE_ASYMMETRY * turn * HIP_FORWARD_DIR;
}

void applyLateralBalance(PoseMap &p, double t) {
  if (!bodyImu)
    return;

  const double *rpy = bodyImu->getRollPitchYaw();
  if (!rpy)
    return;

  static bool havePreviousRoll = false;
  static double previousRoll = 0.0;
  static double previousTime = 0.0;

  const double roll = rpy[0];
  const double dt = havePreviousRoll ? t - previousTime : 0.0;
  const double rollRate = dt > 0.0 ? (roll - previousRoll) / dt : 0.0;
  const double correction = clamp(-(LATERAL_BALANCE_ROLL_GAIN * roll +
                                    LATERAL_BALANCE_RATE_GAIN * rollRate),
                                  -LATERAL_BALANCE_MAX, LATERAL_BALANCE_MAX);

  p["left_hip_roll"] += correction * ROLL_TO_LEFT_DIR;
  p["right_hip_roll"] += correction * ROLL_TO_LEFT_DIR;
  p["left_ankle_roll"] -= 0.75 * correction * ROLL_TO_LEFT_DIR;
  p["right_ankle_roll"] -= 0.75 * correction * ROLL_TO_LEFT_DIR;

  previousRoll = roll;
  previousTime = t;
  havePreviousRoll = true;
}

PoseMap directStandPose() {
  PoseMap p;

  p["neck_yaw"] = 0.0;

  p["waist_yaw"] = 0.0;
  p["waist_pitch"] = 0.0;

  p["left_shoulder_pitch"] = 0.0;
  p["right_shoulder_pitch"] = 0.0;
  p["left_shoulder_roll"] = 0.08;
  p["right_shoulder_roll"] = -0.08;
  p["left_shoulder_yaw"] = 0.0;
  p["right_shoulder_yaw"] = 0.0;
  p["left_elbow_pitch"] = 0.50;
  p["right_elbow_pitch"] = 0.50;

  p["left_hip_roll"] = 0.0;
  p["right_hip_roll"] = 0.0;

  p["left_hip_pitch"] = -0.04 * HIP_FORWARD_DIR;
  p["right_hip_pitch"] = -0.04 * HIP_FORWARD_DIR;

  p["left_knee_pitch"] = 0.08;
  p["right_knee_pitch"] = 0.08;

  p["left_ankle_pitch"] = 0.12;
  p["right_ankle_pitch"] = 0.12;

  p["left_ankle_roll"] = 0.0;
  p["right_ankle_roll"] = 0.0;

  return p;
}

PoseMap shiftLeftPose() {
  PoseMap p = directStandPose();
  double r = HIP_ROLL_SHIFT * ROLL_TO_LEFT_DIR;

  p["left_hip_roll"] = r;
  p["right_hip_roll"] = r;

  p["left_ankle_roll"] = -ANKLE_ROLL_SUPPORT * ROLL_TO_LEFT_DIR;
  p["right_ankle_roll"] = -ANKLE_ROLL_PREP * ROLL_TO_LEFT_DIR;

  p["left_hip_pitch"] = SHIFT_SUPPORT_HIP_FORWARD * HIP_FORWARD_DIR;
  p["right_hip_pitch"] = SHIFT_TRAILING_HIP_BACK * HIP_FORWARD_DIR;
  p["left_knee_pitch"] = SUPPORT_KNEE;
  p["right_knee_pitch"] = SWING_PREP_KNEE;
  p["left_ankle_pitch"] = LAND_SUPPORT_ANKLE * HIP_FORWARD_DIR;
  p["right_ankle_pitch"] = LAND_ANKLE * HIP_FORWARD_DIR;

  setArmSwing(p, ARM_SWING_SHIFT, ARM_ROLL_SHIFT, 0.06);
  setTorso(p, -0.5 * TORSO_YAW, TORSO_PITCH_SHIFT);

  return p;
}

PoseMap swingRightPose() {
  PoseMap p = shiftLeftPose();

  p["left_hip_pitch"] = SUPPORT_HIP_BACK * HIP_FORWARD_DIR;
  p["left_knee_pitch"] = SUPPORT_KNEE;
  p["left_ankle_pitch"] = SUPPORT_ANKLE_PUSH * HIP_FORWARD_DIR;
  p["left_ankle_roll"] = -ANKLE_ROLL_SUPPORT * ROLL_TO_LEFT_DIR;

  p["right_hip_pitch"] = SWING_HIP_FORWARD * HIP_FORWARD_DIR;
  p["right_knee_pitch"] = SWING_KNEE;
  p["right_ankle_pitch"] = SWING_ANKLE_LIFT * HIP_FORWARD_DIR;
  p["right_ankle_roll"] = 0.0;

  setArmSwing(p, ARM_SWING, ARM_ROLL_SWING, 0.12);
  setTorso(p, -TORSO_YAW, TORSO_PITCH_WALK);

  return p;
}

PoseMap rightFootDownPose() {
  PoseMap p = shiftLeftPose();

  p["left_hip_pitch"] = LAND_HIP_BACK * HIP_FORWARD_DIR;
  p["left_knee_pitch"] = SUPPORT_KNEE;
  p["left_ankle_pitch"] = LAND_SUPPORT_ANKLE * HIP_FORWARD_DIR;
  p["left_ankle_roll"] = -ANKLE_ROLL_SUPPORT * ROLL_TO_LEFT_DIR;

  p["right_hip_pitch"] = LAND_HIP_FORWARD * HIP_FORWARD_DIR;
  p["right_knee_pitch"] = LAND_KNEE;
  p["right_ankle_pitch"] = LAND_ANKLE * HIP_FORWARD_DIR;
  p["right_ankle_roll"] = 0.0;

  setArmSwing(p, ARM_SWING_LAND, ARM_ROLL_LAND, 0.08);
  setTorso(p, -TORSO_YAW, TORSO_PITCH_WALK);

  return p;
}

PoseMap shiftRightPose() {
  PoseMap p = directStandPose();
  double r = -HIP_ROLL_SHIFT * ROLL_TO_LEFT_DIR;

  p["left_hip_roll"] = r;
  p["right_hip_roll"] = r;

  p["left_ankle_roll"] = ANKLE_ROLL_PREP * ROLL_TO_LEFT_DIR;
  p["right_ankle_roll"] = ANKLE_ROLL_SUPPORT * ROLL_TO_LEFT_DIR;

  p["left_hip_pitch"] = SHIFT_TRAILING_HIP_BACK * HIP_FORWARD_DIR;
  p["right_hip_pitch"] = SHIFT_SUPPORT_HIP_FORWARD * HIP_FORWARD_DIR;
  p["left_knee_pitch"] = SWING_PREP_KNEE;
  p["right_knee_pitch"] = SUPPORT_KNEE;
  p["left_ankle_pitch"] = LAND_ANKLE * HIP_FORWARD_DIR;
  p["right_ankle_pitch"] = LAND_SUPPORT_ANKLE * HIP_FORWARD_DIR;

  setArmSwing(p, -ARM_SWING_SHIFT, ARM_ROLL_SHIFT, 0.06);
  setTorso(p, 0.5 * TORSO_YAW, TORSO_PITCH_SHIFT);

  return p;
}

PoseMap swingLeftPose() {
  PoseMap p = shiftRightPose();

  p["right_hip_pitch"] = SUPPORT_HIP_BACK * HIP_FORWARD_DIR;
  p["right_knee_pitch"] = SUPPORT_KNEE;
  p["right_ankle_pitch"] = SUPPORT_ANKLE_PUSH * HIP_FORWARD_DIR;
  p["right_ankle_roll"] = ANKLE_ROLL_SUPPORT * ROLL_TO_LEFT_DIR;

  p["left_hip_pitch"] = SWING_HIP_FORWARD * HIP_FORWARD_DIR;
  p["left_knee_pitch"] = SWING_KNEE;
  p["left_ankle_pitch"] = SWING_ANKLE_LIFT * HIP_FORWARD_DIR;
  p["left_ankle_roll"] = 0.0;

  setArmSwing(p, -ARM_SWING, ARM_ROLL_SWING, 0.12);
  setTorso(p, TORSO_YAW, TORSO_PITCH_WALK);

  return p;
}

PoseMap leftFootDownPose() {
  PoseMap p = shiftRightPose();

  p["right_hip_pitch"] = LAND_HIP_BACK * HIP_FORWARD_DIR;
  p["right_knee_pitch"] = SUPPORT_KNEE;
  p["right_ankle_pitch"] = LAND_SUPPORT_ANKLE * HIP_FORWARD_DIR;
  p["right_ankle_roll"] = ANKLE_ROLL_SUPPORT * ROLL_TO_LEFT_DIR;

  p["left_hip_pitch"] = LAND_HIP_FORWARD * HIP_FORWARD_DIR;
  p["left_knee_pitch"] = LAND_KNEE;
  p["left_ankle_pitch"] = LAND_ANKLE * HIP_FORWARD_DIR;
  p["left_ankle_roll"] = 0.0;

  setArmSwing(p, -ARM_SWING_LAND, ARM_ROLL_LAND, 0.08);
  setTorso(p, TORSO_YAW, TORSO_PITCH_WALK);

  return p;
}

struct LegAngles {
  double hip;
  double knee;
  double ankle;
  double swingWeight;
  double supportWeight;
  double footX;
};

struct FootTarget {
  double x;
  double zDown;
  double pitch;
  double swingWeight;
  double supportWeight;
};

struct IkRawAngles {
  double hip;
  double knee;
  double ankle;
};

struct Vec3 {
  double x;
  double y;
  double z;
};

struct RobotModel {
  double hipLateralOffset;
  double strideHalf;
  double upperBodyMass;
  double hipLinkMass;
  double thighMass;
  double shinMass;
  double footMass;
};

struct WalkState {
  bool haveCom;
  double leftSupport;
  double rightSupport;
  double leftFootY;
  double rightFootY;
  double supportTargetY;
  double comY;
  double comToSupportY;
};

// Compact model parameters lifted from MechaHumanoid21.proto. These values are
// used for step timing and lateral COM estimates; sagittal knee/ankle motion is
// driven explicitly by the gait phase below.
const RobotModel ROBOT_MODEL = {
  0.145,  // hipLateralOffset
  0.180,  // strideHalf
  4.20,   // upperBodyMass
  0.05,   // hipLinkMass
  0.80,   // thighMass
  0.64,   // shinMass, including the ankle pitch link
  0.80    // footMass
};

const double STRIDE_HALF = ROBOT_MODEL.strideHalf;

double supportWeightForPhase(double phase) {
  phase = wrap01(phase);
  if (phase >= STANCE_FRACTION)
    return 0.0;

  const double stancePhase = phase / STANCE_FRACTION;
  return rampUp(stancePhase / 0.26) * rampDown((stancePhase - 0.74) / 0.26);
}

double swingWeightForPhase(double phase) {
  phase = wrap01(phase);
  if (phase < STANCE_FRACTION)
    return 0.0;

  const double swingPhase = (phase - STANCE_FRACTION) / SWING_FRACTION;
  return softBell(swingPhase);
}

FootTarget footTargetForPhase(double phase) {
  phase = wrap01(phase);
  FootTarget out;
  out.swingWeight = swingWeightForPhase(phase);
  out.supportWeight = supportWeightForPhase(phase);

  if (phase < STANCE_FRACTION) {
    const double stancePhase = phase / STANCE_FRACTION;
    const double u = rollingEase(stancePhase);
    out.x = lerp(STRIDE_HALF, -STRIDE_HALF, u);
  } else {
    const double swingPhase = (phase - STANCE_FRACTION) / SWING_FRACTION;
    const double u = rollingEase((swingPhase - 0.10) / 0.82);
    out.x = lerp(-STRIDE_HALF, STRIDE_HALF, u);
  }

  const double maxReach = IK_THIGH_LENGTH + IK_SHIN_LENGTH - 0.004;
  const double reachLimitedDown = std::sqrt(std::max(0.001, maxReach * maxReach - out.x * out.x));
  out.zDown = IK_STANCE_ANKLE_DOWN < reachLimitedDown ? IK_STANCE_ANKLE_DOWN : reachLimitedDown;
  out.zDown -= IK_SWING_FOOT_LIFT * out.swingWeight;
  out.pitch = IK_SWING_FOOT_PITCH * out.swingWeight;
  return out;
}

IkRawAngles solveRawSagittalIk(double x, double zDown, double footPitch) {
  const double minReach = std::fabs(IK_SHIN_LENGTH - IK_THIGH_LENGTH) + 0.004;
  const double maxReach = IK_THIGH_LENGTH + IK_SHIN_LENGTH - 0.004;
  double reach = std::sqrt(x * x + zDown * zDown);
  reach = clamp(reach, minReach, maxReach);

  const double cosKnee = clamp((reach * reach - IK_THIGH_LENGTH * IK_THIGH_LENGTH -
                                IK_SHIN_LENGTH * IK_SHIN_LENGTH) /
                                 (2.0 * IK_THIGH_LENGTH * IK_SHIN_LENGTH),
                               -1.0, 1.0);
  IkRawAngles out;
  out.knee = std::acos(cosKnee);
  out.hip = std::atan2(x, zDown) -
            std::atan2(IK_SHIN_LENGTH * std::sin(out.knee),
                       IK_THIGH_LENGTH + IK_SHIN_LENGTH * std::cos(out.knee));
  // Knee pitch is mounted on the opposite axis from hip/ankle pitch in the
  // PROTO, so foot pitch is hip - knee + ankle.
  out.ankle = footPitch - out.hip + out.knee;
  return out;
}

double groundLockedAnkleForPose(double hip, double knee, double footPitch) {
  return clamp(footPitch - hip + knee, -ANKLE_PITCH_LIMIT, ANKLE_PITCH_LIMIT);
}

double swingFootPitchForPhase(double phase) {
  phase = wrap01(phase);
  if (phase < STANCE_FRACTION)
    return GROUND_FOOT_PITCH;

  const double swingPhase = (phase - STANCE_FRACTION) / SWING_FRACTION;
  if (swingPhase < 0.24) {
    const double u = rollingEase(swingPhase / 0.24);
    return lerp(GROUND_FOOT_PITCH, SWING_FOOT_TOE_UP_PITCH, u);
  }
  if (swingPhase < 0.62) {
    const double u = rollingEase((swingPhase - 0.24) / 0.38);
    return lerp(SWING_FOOT_TOE_UP_PITCH, SWING_FOOT_CLEARANCE_PITCH, u);
  }

  const double u = rollingEase((swingPhase - 0.62) / 0.38);
  return lerp(SWING_FOOT_CLEARANCE_PITCH, SWING_FOOT_LANDING_PITCH, u);
}

double swingAnklePitchForPhase(double phase) {
  const double swingPhase = (wrap01(phase) - STANCE_FRACTION) / SWING_FRACTION;
  if (swingPhase < 0.20) {
    const double u = rollingEase(swingPhase / 0.20);
    return lerp(TOE_OFF_ANKLE, SWING_ANKLE_DOWN_EARLY, u);
  }
  if (swingPhase < 0.62) {
    const double u = rollingEase((swingPhase - 0.20) / 0.42);
    return lerp(SWING_ANKLE_DOWN_EARLY, SWING_ANKLE_DOWN_MID, u);
  }

  const double u = rollingEase((swingPhase - 0.62) / 0.38);
  return lerp(SWING_ANKLE_DOWN_MID, SWING_ANKLE_DOWN_LANDING, u);
}

// One gait cycle starts at heel strike. The sagittal joints are driven as a
// visible rocker sequence: heel strike, body rolling over the planted ankle,
// toe-off, swing clearance, then knee extension into the next heel strike.
LegAngles curveLegAnglesForPhase(double phase) {
  const FootTarget target = footTargetForPhase(phase);

  LegAngles out;
  out.swingWeight = target.swingWeight;
  out.supportWeight = target.supportWeight;
  out.footX = target.x;

  phase = wrap01(phase);
  if (phase < STANCE_FRACTION) {
    const double stancePhase = phase / STANCE_FRACTION;

    if (stancePhase < 0.18) {
      const double u = rollingEase(stancePhase / 0.18);
      out.hip = lerp(HEEL_STRIKE_HIP, 0.38, u);
      out.knee = lerp(SWING_RECOVERY_KNEE, LOADING_KNEE, u);
      out.ankle = lerp(HEEL_STRIKE_ANKLE, FOOT_FLAT_ANKLE, u);
    } else if (stancePhase < 0.62) {
      const double u = rollingEase((stancePhase - 0.18) / 0.44);
      out.hip = lerp(0.38, MID_STANCE_HIP, u);
      out.knee = lerp(LOADING_KNEE, MID_STANCE_KNEE, u);
      out.ankle = lerp(FOOT_FLAT_ANKLE, MID_STANCE_ANKLE, u);
    } else if (stancePhase < 0.84) {
      const double u = rollingEase((stancePhase - 0.62) / 0.22);
      out.hip = lerp(MID_STANCE_HIP, -0.22, u);
      out.knee = lerp(MID_STANCE_KNEE, TOE_OFF_KNEE, u);
      out.ankle = lerp(MID_STANCE_ANKLE, TOE_OFF_ANKLE, u);
    } else {
      const double u = rollingEase((stancePhase - 0.84) / 0.16);
      out.hip = lerp(-0.22, TOE_OFF_HIP, u);
      out.knee = lerp(TOE_OFF_KNEE, 0.62, u);
      out.ankle = TOE_OFF_ANKLE;
    }
  } else {
    const double swingPhase = (phase - STANCE_FRACTION) / SWING_FRACTION;

    if (swingPhase < 0.24) {
      const double u = rollingEase(swingPhase / 0.24);
      out.hip = lerp(TOE_OFF_HIP, EARLY_SWING_HIP, u);
      out.knee = lerp(0.62, PEAK_SWING_KNEE, u);
      out.ankle = lerp(TOE_OFF_ANKLE, SWING_TOE_UP_ANKLE, u);
    } else if (swingPhase < 0.62) {
      const double u = rollingEase((swingPhase - 0.24) / 0.38);
      out.hip = lerp(EARLY_SWING_HIP, LATE_SWING_HIP, u);
      out.knee = lerp(PEAK_SWING_KNEE, 0.70, u);
      out.ankle = lerp(SWING_TOE_UP_ANKLE, SWING_CLEARANCE_ANKLE, u);
    } else {
      const double u = rollingEase((swingPhase - 0.62) / 0.38);
      out.hip = lerp(LATE_SWING_HIP, HEEL_STRIKE_HIP, u);
      out.knee = lerp(0.70, SWING_RECOVERY_KNEE, u);
      out.ankle = lerp(SWING_CLEARANCE_ANKLE, HEEL_STRIKE_ANKLE, u);
    }
  }

  out.hip = clamp(out.hip, -1.2, 1.2);
  out.knee = clamp(out.knee, 0.0, 2.2);
  out.ankle = clamp(out.ankle, -ANKLE_PITCH_LIMIT, ANKLE_PITCH_LIMIT);
  return out;
}

LegAngles ikLegAnglesForPhase(double phase) {
  const FootTarget target = footTargetForPhase(phase);
  const IkRawAngles neutral = solveRawSagittalIk(0.0, IK_STANCE_ANKLE_DOWN, 0.0);
  const IkRawAngles raw = solveRawSagittalIk(target.x, target.zDown, target.pitch);

  LegAngles out;
  out.hip = IK_NEUTRAL_HIP + (raw.hip - neutral.hip);
  out.knee = IK_NEUTRAL_KNEE + (raw.knee - neutral.knee);
  out.ankle = IK_NEUTRAL_ANKLE + (raw.ankle - neutral.ankle);
  out.swingWeight = target.swingWeight;
  out.supportWeight = target.supportWeight;
  out.footX = target.x;

  out.hip = clamp(out.hip, -1.2, 1.2);
  out.knee = clamp(out.knee, 0.0, 2.2);
  out.ankle = clamp(out.ankle, -ANKLE_PITCH_LIMIT, ANKLE_PITCH_LIMIT);
  return out;
}

LegAngles legAnglesForPhase(double phase) {
  const LegAngles curve = curveLegAnglesForPhase(phase);
  const LegAngles ik = ikLegAnglesForPhase(phase);

  LegAngles out = curve;
  out.hip = lerp(curve.hip, ik.hip, IK_BLEND);
  out.knee = lerp(curve.knee, ik.knee, IK_BLEND);
  out.ankle = lerp(curve.ankle, ik.ankle, IK_BLEND);
  out.swingWeight = curve.swingWeight;
  out.supportWeight = curve.supportWeight;
  out.footX = curve.footX;
  if (wrap01(phase) < STANCE_FRACTION) {
    out.ankle = groundLockedAnkleForPose(out.hip, out.knee, GROUND_FOOT_PITCH);
  } else {
    out.ankle = clamp(swingAnklePitchForPhase(phase), -ANKLE_PITCH_LIMIT, ANKLE_PITCH_LIMIT);
  }
  return out;
}

Vec3 makeVec3(double x, double y, double z) {
  Vec3 out;
  out.x = x;
  out.y = y;
  out.z = z;
  return out;
}

Vec3 vec3FromArray(const double *v) {
  if (!v)
    return makeVec3(0.0, 0.0, 0.0);
  return makeVec3(v[0], v[1], v[2]);
}

Vec3 subtractVec3(const Vec3 &a, const Vec3 &b) {
  return makeVec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

double dotVec3(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 orientationAxis(const double *orientation, int axis, const Vec3 &fallback) {
  if (!orientation)
    return fallback;
  return makeVec3(orientation[axis], orientation[axis + 3], orientation[axis + 6]);
}

double localLateralY(const Vec3 &point, const Vec3 &origin, const Vec3 &sideAxis) {
  return dotVec3(subtractVec3(point, origin), sideAxis);
}

void addLateralMass(double localY, double mass, double &weightedY, double &totalMass) {
  weightedY += localY * mass;
  totalMass += mass;
}

void addLegMassEstimate(double hipY, double footY, double &weightedY, double &totalMass) {
  addLateralMass(hipY, ROBOT_MODEL.hipLinkMass, weightedY, totalMass);
  addLateralMass(lerp(hipY, footY, 0.25), ROBOT_MODEL.thighMass, weightedY, totalMass);
  addLateralMass(lerp(hipY, footY, 0.65), ROBOT_MODEL.shinMass, weightedY, totalMass);
  addLateralMass(footY, ROBOT_MODEL.footMass, weightedY, totalMass);
}

WalkState estimateWalkState(Supervisor *robot, double walkTime) {
  WalkState state;
  state.haveCom = false;
  state.leftSupport = supportWeightForPhase(leftPhaseForWalkTime(walkTime));
  state.rightSupport = supportWeightForPhase(rightPhaseForWalkTime(walkTime));
  state.leftFootY = ROBOT_MODEL.hipLateralOffset;
  state.rightFootY = -ROBOT_MODEL.hipLateralOffset;
  state.supportTargetY = 0.0;
  state.comY = 0.0;
  state.comToSupportY = 0.0;

  if (!robot)
    return state;

  Node *self = robot->getSelf();
  const double *rootValues = self ? self->getPosition() : nullptr;
  if (!rootValues)
    return state;

  const Vec3 root = vec3FromArray(rootValues);
  const Vec3 sideAxis = orientationAxis(self->getOrientation(), 1, makeVec3(0.0, 1.0, 0.0));

  if (leftFootGps)
    state.leftFootY = localLateralY(vec3FromArray(leftFootGps->getValues()), root, sideAxis);
  if (rightFootGps)
    state.rightFootY = localLateralY(vec3FromArray(rightFootGps->getValues()), root, sideAxis);

  double weightedY = 0.0;
  double totalMass = 0.0;
  addLateralMass(0.0, ROBOT_MODEL.upperBodyMass, weightedY, totalMass);
  addLegMassEstimate(ROBOT_MODEL.hipLateralOffset, state.leftFootY, weightedY, totalMass);
  addLegMassEstimate(-ROBOT_MODEL.hipLateralOffset, state.rightFootY, weightedY, totalMass);

  if (totalMass <= 0.0)
    return state;

  const double supportTotal = state.leftSupport + state.rightSupport;
  double rawSupportY = 0.5 * (state.leftFootY + state.rightFootY);
  if (supportTotal > 0.05)
    rawSupportY = (state.leftFootY * state.leftSupport + state.rightFootY * state.rightSupport) / supportTotal;

  state.haveCom = true;
  state.comY = weightedY / totalMass;
  state.supportTargetY = clamp(rawSupportY, -COM_TARGET_LATERAL_LIMIT, COM_TARGET_LATERAL_LIMIT);
  state.comToSupportY = state.supportTargetY - state.comY;
  return state;
}

void applyComBalance(PoseMap &p, const WalkState &state, double t) {
  if (!state.haveCom)
    return;

  static bool havePreviousCorrection = false;
  static double previousTime = 0.0;
  static double filteredCorrection = 0.0;

  double error = state.comToSupportY;
  if (std::fabs(error) <= COM_BALANCE_DEADBAND) {
    error = 0.0;
  } else {
    error = error > 0.0 ? error - COM_BALANCE_DEADBAND : error + COM_BALANCE_DEADBAND;
  }

  const double targetCorrection = clamp(COM_BALANCE_GAIN * error, -COM_BALANCE_MAX, COM_BALANCE_MAX);
  const double dt = havePreviousCorrection ? t - previousTime : 0.0;
  const double alpha = dt > 0.0 ? 1.0 - std::exp(-dt / COM_BALANCE_FILTER_TIME) : 1.0;
  filteredCorrection = lerp(filteredCorrection, targetCorrection, alpha);

  p["left_hip_roll"] += filteredCorrection * ROLL_TO_LEFT_DIR;
  p["right_hip_roll"] += filteredCorrection * ROLL_TO_LEFT_DIR;
  p["left_ankle_roll"] -= 0.60 * filteredCorrection * ROLL_TO_LEFT_DIR;
  p["right_ankle_roll"] -= 0.60 * filteredCorrection * ROLL_TO_LEFT_DIR;

  previousTime = t;
  havePreviousCorrection = true;
}

void limitLateralPose(PoseMap &p) {
  p["left_hip_roll"] = clamp(poseValue(p, "left_hip_roll"), -HIP_ROLL_LIMIT, HIP_ROLL_LIMIT);
  p["right_hip_roll"] = clamp(poseValue(p, "right_hip_roll"), -HIP_ROLL_LIMIT, HIP_ROLL_LIMIT);
  p["left_ankle_roll"] = clamp(poseValue(p, "left_ankle_roll"), -ANKLE_ROLL_LIMIT, ANKLE_ROLL_LIMIT);
  p["right_ankle_roll"] = clamp(poseValue(p, "right_ankle_roll"), -ANKLE_ROLL_LIMIT, ANKLE_ROLL_LIMIT);
}

PoseMap naturalWalkPose(double walkTime) {
  PoseMap p = directStandPose();

  const double leftPhase = leftPhaseForWalkTime(walkTime);
  const double rightPhase = rightPhaseForWalkTime(walkTime);
  const LegAngles left = legAnglesForPhase(leftPhase);
  const LegAngles right = legAnglesForPhase(rightPhase);

  p["left_hip_pitch"] = left.hip * HIP_FORWARD_DIR;
  p["right_hip_pitch"] = right.hip * HIP_FORWARD_DIR;
  p["left_knee_pitch"] = left.knee;
  p["right_knee_pitch"] = right.knee;
  p["left_ankle_pitch"] = left.ankle * HIP_FORWARD_DIR;
  p["right_ankle_pitch"] = right.ankle * HIP_FORWARD_DIR;

  const double leftSupport = left.supportWeight;
  const double rightSupport = right.supportWeight;
  const double supportBalance = clamp(leftSupport - rightSupport, -1.0, 1.0);
  const double lateral = HIP_ROLL_SHIFT * supportBalance * ROLL_TO_LEFT_DIR;
  p["left_hip_roll"] = lateral - SWING_HIP_ROLL_CENTER * left.swingWeight * ROLL_TO_LEFT_DIR;
  p["right_hip_roll"] = lateral + SWING_HIP_ROLL_CENTER * right.swingWeight * ROLL_TO_LEFT_DIR;
  p["left_ankle_roll"] = -ANKLE_ROLL_SUPPORT * supportBalance * ROLL_TO_LEFT_DIR;
  p["right_ankle_roll"] = -ANKLE_ROLL_SUPPORT * supportBalance * ROLL_TO_LEFT_DIR;

  const double stepSignal = clamp((left.footX - right.footX) / (2.0 * STRIDE_HALF), -1.0, 1.0);
  const double swingEnergy = clamp(left.swingWeight + right.swingWeight, 0.0, 1.0);
  const double bodyFollow = std::sin(2.0 * M_PI * (leftPhase + 0.02));
  const double armSignal = std::cos(2.0 * M_PI * rightPhase);
  const double torsoTwist = TORSO_YAW * stepSignal;
  setArmSwing(p, ARM_SWING * armSignal, 0.0, 0.05);
  p["left_shoulder_yaw"] = 0.0;
  p["right_shoulder_yaw"] = 0.0;
  p["waist_yaw"] = torsoTwist;
  p["neck_yaw"] = 0.0;
  p["waist_pitch"] = clamp(TORSO_FORWARD_LEAN - 0.006 * swingEnergy + 0.004 * bodyFollow,
                           -0.10, -0.025);

  return p;
}

void applyDirectPose(const PoseMap &p) {
  setMotor("neck_yaw", poseValue(p, "neck_yaw"), -1.2, 1.2);

  setMotor("waist_yaw", poseValue(p, "waist_yaw"), -0.6, 0.6);
  setMotor("waist_pitch", poseValue(p, "waist_pitch"), -0.4, 0.4);

  setMotor("left_shoulder_pitch", L_SHOULDER_PITCH_SIGN * poseValue(p, "left_shoulder_pitch"), -1.8, 1.8);
  setMotor("right_shoulder_pitch", R_SHOULDER_PITCH_SIGN * poseValue(p, "right_shoulder_pitch"), -1.8, 1.8);

  setMotor("left_shoulder_roll", poseValue(p, "left_shoulder_roll"), -1.2, 1.2);
  setMotor("right_shoulder_roll", poseValue(p, "right_shoulder_roll"), -1.2, 1.2);

  setMotor("left_shoulder_yaw", poseValue(p, "left_shoulder_yaw"), -1.5, 1.5);
  setMotor("right_shoulder_yaw", poseValue(p, "right_shoulder_yaw"), -1.5, 1.5);

  setMotor("left_elbow_pitch", poseValue(p, "left_elbow_pitch"), 0.0, 2.2);
  setMotor("right_elbow_pitch", poseValue(p, "right_elbow_pitch"), 0.0, 2.2);

  setMotor("left_hip_roll", L_HIP_ROLL_SIGN * poseValue(p, "left_hip_roll"), -0.7, 0.7);
  setMotor("right_hip_roll", R_HIP_ROLL_SIGN * poseValue(p, "right_hip_roll"), -0.7, 0.7);

  setMotor("left_hip_pitch", L_HIP_PITCH_SIGN * poseValue(p, "left_hip_pitch"), -1.2, 1.2);
  setMotor("right_hip_pitch", R_HIP_PITCH_SIGN * poseValue(p, "right_hip_pitch"), -1.2, 1.2);

  setMotor("left_knee_pitch", L_KNEE_SIGN * poseValue(p, "left_knee_pitch"), 0.0, 2.2);
  setMotor("right_knee_pitch", R_KNEE_SIGN * poseValue(p, "right_knee_pitch"), 0.0, 2.2);

  setMotor("left_ankle_pitch", L_ANKLE_PITCH_SIGN * poseValue(p, "left_ankle_pitch"), -ANKLE_PITCH_LIMIT, ANKLE_PITCH_LIMIT);
  setMotor("right_ankle_pitch", R_ANKLE_PITCH_SIGN * poseValue(p, "right_ankle_pitch"), -ANKLE_PITCH_LIMIT, ANKLE_PITCH_LIMIT);

  setMotor("left_ankle_roll", L_ANKLE_ROLL_SIGN * poseValue(p, "left_ankle_roll"), -0.35, 0.35);
  setMotor("right_ankle_roll", R_ANKLE_ROLL_SIGN * poseValue(p, "right_ankle_roll"), -0.35, 0.35);
}
 
void applyWalkPose(Supervisor *robot) {
  double t = robot->getTime();
  static double previousApplyTime = -1.0;

  if (RESCUE_STAND_ONLY || t < 2.0) {
    PoseMap standPose = directStandPose();
    lastAppliedPose = standPose;
    applyDirectPose(standPose);
    previousApplyTime = t;
    return;
  }

  double startBlend = smoothStep(clamp((t - 2.0) / START_BLEND_TIME, 0.0, 1.0));

  PoseMap stand = directStandPose();
  PoseMap target = naturalWalkPose(t - 2.0);
  WalkState state = estimateWalkState(robot, t - 2.0);
  applyComBalance(target, state, t);
  applyLateralBalance(target, t);
  applyTurnToPose(target);
  limitLateralPose(target);

  PoseMap desiredPose = mixPose(stand, target, startBlend);
  limitLateralPose(desiredPose);
  const double dt = previousApplyTime >= 0.0 ? t - previousApplyTime : 0.0;
  const double filterAlpha = dt > 0.0 ? 1.0 - std::exp(-dt / COMMAND_FILTER_TIME) : 1.0;
  PoseMap finalPose = filterPose(lastAppliedPose, desiredPose, filterAlpha);
  limitLateralPose(finalPose);
  previousApplyTime = t;
  lastAppliedPose = finalPose;
  applyDirectPose(finalPose);
}

void debugWalkState(Supervisor *robot) {
  static double lastPrintTime = -1.0;
  static bool haveStartRoot = false;
  static bool havePreviousRoot = false;
  static bool warnedNoForward = false;
  static double startRoot[3] = {0.0, 0.0, 0.0};
  static double previousRoot[3] = {0.0, 0.0, 0.0};
  double t = robot->getTime();

  if (lastPrintTime >= 0.0 && t - lastPrintTime < 0.2)
    return;
  lastPrintTime = t;

  Node *self = robot->getSelf();

  const double *rootPos = self ? self->getPosition() : nullptr;
  const double *rootOrientation = self ? self->getOrientation() : nullptr;
  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;
  double stepDx = 0.0;
  double stepDy = 0.0;
  double stepDz = 0.0;
  double forwardDelta = 0.0;
  double stepForwardDelta = 0.0;
  double forwardAxis[3] = {ROBOT_LOCAL_FORWARD_X, 0.0, 0.0};
  if (rootOrientation) {
    forwardAxis[0] = ROBOT_LOCAL_FORWARD_X * rootOrientation[0];
    forwardAxis[1] = ROBOT_LOCAL_FORWARD_X * rootOrientation[3];
    forwardAxis[2] = ROBOT_LOCAL_FORWARD_X * rootOrientation[6];
  }
  if (rootPos && !haveStartRoot) {
    startRoot[0] = rootPos[0];
    startRoot[1] = rootPos[1];
    startRoot[2] = rootPos[2];
    haveStartRoot = true;
  }
  if (rootPos && haveStartRoot) {
    dx = rootPos[0] - startRoot[0];
    dy = rootPos[1] - startRoot[1];
    dz = rootPos[2] - startRoot[2];
    forwardDelta = dx * forwardAxis[0] + dy * forwardAxis[1] + dz * forwardAxis[2];
  }
  if (rootPos && havePreviousRoot) {
    stepDx = rootPos[0] - previousRoot[0];
    stepDy = rootPos[1] - previousRoot[1];
    stepDz = rootPos[2] - previousRoot[2];
    stepForwardDelta = stepDx * forwardAxis[0] + stepDy * forwardAxis[1] + stepDz * forwardAxis[2];
  }
  if (rootPos) {
    previousRoot[0] = rootPos[0];
    previousRoot[1] = rootPos[1];
    previousRoot[2] = rootPos[2];
    havePreviousRoot = true;
  }

  std::string supportFoot = "standing";
  std::string swingFoot = "none";
  if (t >= 2.0) {
    const double walkTime = t - 2.0;
    const double leftPhase = leftPhaseForWalkTime(walkTime);
    const double rightPhase = rightPhaseForWalkTime(walkTime);

    if (leftPhase >= STANCE_FRACTION) {
      supportFoot = "right";
      swingFoot = "left";
    } else if (rightPhase >= STANCE_FRACTION) {
      supportFoot = "left";
      swingFoot = "right";
    } else {
      supportFoot = "double";
      swingFoot = "none";
    }
  }

  std::cout << std::fixed << std::setprecision(3)
            << "[walk-debug] t=" << t;
  if (rootPos) {
    std::cout << " root=(" << rootPos[0] << ", " << rootPos[1] << ", " << rootPos[2] << ")"
              << " dx=" << dx
              << " forward=" << forwardDelta
              << " rootDelta=(" << dx << ", " << dy << ", " << dz << ")"
              << " stepDelta=(" << stepDx << ", " << stepDy << ", " << stepDz << ")"
              << " stepForward=" << stepForwardDelta;
  } else {
    std::cout << " root=(unavailable)";
  }
  std::cout << " support=" << supportFoot
            << " swing=" << swingFoot
            << " turn=" << turnCommand
            << " L(h/k/a)=" << poseValue(lastAppliedPose, "left_hip_pitch") << "/"
            << poseValue(lastAppliedPose, "left_knee_pitch") << "/"
            << poseValue(lastAppliedPose, "left_ankle_pitch")
            << " R(h/k/a)=" << poseValue(lastAppliedPose, "right_hip_pitch") << "/"
            << poseValue(lastAppliedPose, "right_knee_pitch") << "/"
            << poseValue(lastAppliedPose, "right_ankle_pitch")
            << std::endl;

  if (rootPos && !warnedNoForward && t >= 5.0 && forwardDelta < 0.03) {
    std::cout << "[walk-debug] WARNING: forward progress is still small/backward after 5s. "
              << "Check HIP_FORWARD_DIR and the L/R hip/knee/ankle pitch sign constants." << std::endl;
    warnedNoForward = true;
  }
}

int main() {
  Supervisor robot;
  int timestep = static_cast<int>(robot.getBasicTimeStep());

  const std::vector<std::string> motorNames = {
    "neck_yaw",
    "waist_yaw",
    "waist_pitch",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow_pitch",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow_pitch",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee_pitch",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee_pitch",
    "right_ankle_pitch",
    "right_ankle_roll"
  };

  for (size_t i = 0; i < motorNames.size(); ++i)
    registerMotor(&robot, timestep, motorNames[i]);

  leftFootGps = robot.getGPS("left_foot_gps");
  if (leftFootGps)
    leftFootGps->enable(timestep);
  else
    std::cout << "WARNING missing sensor: left_foot_gps" << std::endl;

  rightFootGps = robot.getGPS("right_foot_gps");
  if (rightFootGps)
    rightFootGps->enable(timestep);
  else
    std::cout << "WARNING missing sensor: right_foot_gps" << std::endl;

  bodyImu = robot.getInertialUnit("body_imu");
  if (bodyImu)
    bodyImu->enable(timestep);
  else
    std::cout << "WARNING missing inertial unit: body_imu" << std::endl;

  std::cout << "straight walking enabled: turn control disabled" << std::endl;

  while (robot.step(timestep) != -1) {
    applyWalkPose(&robot);
    if (DEBUG_WALK)
      debugWalkState(&robot);
  }

  return 0;
}
