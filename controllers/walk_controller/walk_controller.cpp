#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

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
const double MOTOR_VELOCITY = 4.2;
const double MOTOR_TORQUE = 96.0;
const double ANKLE_MOTOR_VELOCITY = 3.6;
const double ANKLE_MOTOR_TORQUE = 45.0;

const double GAIT_CYCLE_TIME = 1.58;
const double STANCE_FRACTION = 0.68;
const double SWING_FRACTION = 1.0 - STANCE_FRACTION;
const double START_BLEND_TIME = 1.40;
const double COMMAND_FILTER_TIME = 0.075;
const double SHIFT_TIME = 0.20;
const double SWING_TIME = 0.31;
const double DOWN_TIME = 0.15;
const bool RESCUE_STAND_ONLY = false;
const bool DEBUG_WALK = false;

const double ARM_SWING = 0.58;
const double ARM_SWING_LAND = 0.34;
const double ARM_SWING_SHIFT = 0.18;
const double ARM_ROLL_SWING = 0.24;
const double ARM_ROLL_LAND = 0.18;
const double ARM_ROLL_SHIFT = 0.14;
const double ARM_ELBOW_BASE = 0.62;
const double ARM_ELBOW_BACK_GAIN = 0.08;
const double TORSO_YAW = 0.036;
const double TORSO_FORWARD_LEAN = 0.060;
const double TORSO_PITCH_SHIFT = 0.020;
const double TORSO_PITCH_WALK = 0.042;

const double HIP_ROLL_SHIFT = 0.055;
const double ANKLE_ROLL_SUPPORT = 0.075;
const double ANKLE_ROLL_PREP = 0.030;
const double SWING_HIP_ROLL_CENTER = 0.045;
const double SHIFT_SUPPORT_HIP_FORWARD = 0.08;
const double SHIFT_TRAILING_HIP_BACK = -0.14;
const double SUPPORT_HIP_BACK = -0.18;
const double SWING_HIP_FORWARD = 0.66;
const double LAND_HIP_BACK = -0.16;
const double LAND_HIP_FORWARD = 0.52;
const double SUPPORT_KNEE = 0.16;
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

const double HEEL_STRIKE_HIP = 0.54;
const double MID_STANCE_HIP = 0.02;
const double TOE_OFF_HIP = -0.18;
const double EARLY_SWING_HIP = 0.32;
const double LATE_SWING_HIP = 0.76;
const double LOADING_KNEE = 0.26;
const double MID_STANCE_KNEE = 0.12;
const double TOE_OFF_KNEE = 0.28;
const double PEAK_SWING_KNEE = 1.02;
const double SWING_RECOVERY_KNEE = 0.44;
const double HEEL_STRIKE_ANKLE = -0.08;
const double FOOT_FLAT_ANKLE = -0.02;
const double TOE_OFF_ANKLE = 0.02;
const double SWING_CLEARANCE_ANKLE = -0.24;

const double DEFAULT_TURN_COMMAND = 0.0;
const double TURN_COMMAND_MAX = 0.75;
const double TURN_WAIST_YAW = 0.10;
const double TURN_NECK_YAW = -0.04;
const double TURN_STRIDE_ASYMMETRY = 0.06;
const double TURN_LEAN_ROLL = 0.018;
const double TURN_ANKLE_ROLL = 0.025;

struct MotorHandle {
  Motor *motor;
  PositionSensor *sensor;
};

typedef std::map<std::string, double> PoseMap;

std::map<std::string, MotorHandle> motors;
PoseMap lastAppliedPose;
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

double cosineEase(double x) {
  x = clamp(x, 0.0, 1.0);
  return 0.5 - 0.5 * std::cos(M_PI * x);
}

double wrap01(double x) {
  x = std::fmod(x, 1.0);
  if (x < 0.0)
    x += 1.0;
  return x;
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
  double forwardElbow = ARM_ELBOW_BASE + elbowGain * effort;
  double backwardElbow = ARM_ELBOW_BASE + ARM_ELBOW_BACK_GAIN * effort;

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

PoseMap directStandPose() {
  PoseMap p;

  p["neck_yaw"] = 0.0;

  p["waist_yaw"] = 0.0;
  p["waist_pitch"] = 0.0;

  p["left_shoulder_pitch"] = 0.0;
  p["right_shoulder_pitch"] = 0.0;
  p["left_shoulder_roll"] = 0.16;
  p["right_shoulder_roll"] = -0.16;
  p["left_shoulder_yaw"] = 0.0;
  p["right_shoulder_yaw"] = 0.0;
  p["left_elbow_pitch"] = 0.45;
  p["right_elbow_pitch"] = 0.45;

  p["left_hip_roll"] = 0.0;
  p["right_hip_roll"] = 0.0;

  p["left_hip_pitch"] = -0.04 * HIP_FORWARD_DIR;
  p["right_hip_pitch"] = -0.04 * HIP_FORWARD_DIR;

  p["left_knee_pitch"] = 0.08;
  p["right_knee_pitch"] = 0.08;

  p["left_ankle_pitch"] = -0.04;
  p["right_ankle_pitch"] = -0.04;

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
};

// One gait cycle starts at heel strike. A longer stance phase leaves a short
// double-support window and avoids the old toe-tapping swing-first motion.
LegAngles legAnglesForPhase(double phase) {
  LegAngles out;
  out.hip = HEEL_STRIKE_HIP;
  out.knee = SUPPORT_KNEE;
  out.ankle = HEEL_STRIKE_ANKLE;
  out.swingWeight = 0.0;
  out.supportWeight = 0.0;

  if (phase < STANCE_FRACTION) {
    const double stancePhase = phase / STANCE_FRACTION;
    const double stanceEase = cosineEase(stancePhase);
    const double loadPulse = std::sin(M_PI * stancePhase);
    const double pushPulse = cosineEase(stancePhase);

    out.supportWeight = rampUp(stancePhase / 0.12) * rampDown((stancePhase - 0.88) / 0.12);
    out.hip = lerp(HEEL_STRIKE_HIP, TOE_OFF_HIP, stanceEase);
    out.knee = lerp(SUPPORT_KNEE, TOE_OFF_KNEE, pushPulse) + 0.10 * loadPulse;
    out.ankle = lerp(HEEL_STRIKE_ANKLE, TOE_OFF_ANKLE, stanceEase);
  } else {
    const double swingPhase = (phase - STANCE_FRACTION) / SWING_FRACTION;
    const double swingEase = cosineEase(swingPhase);
    const double liftPulse = std::sin(M_PI * swingPhase);
    const double kneeBase = lerp(TOE_OFF_KNEE, SUPPORT_KNEE, swingEase);
    const double ankleBase = lerp(TOE_OFF_ANKLE, HEEL_STRIKE_ANKLE, swingEase);

    out.swingWeight = liftPulse;
    out.hip = lerp(TOE_OFF_HIP, HEEL_STRIKE_HIP, swingEase) +
              (LATE_SWING_HIP - 0.5 * (TOE_OFF_HIP + HEEL_STRIKE_HIP)) * liftPulse;
    out.knee = kneeBase + (PEAK_SWING_KNEE - 0.5 * (TOE_OFF_KNEE + SUPPORT_KNEE)) * liftPulse;
    out.ankle = ankleBase + (SWING_CLEARANCE_ANKLE - 0.5 * (TOE_OFF_ANKLE + HEEL_STRIKE_ANKLE)) * liftPulse;
  }

  return out;
}

PoseMap naturalWalkPose(double walkTime) {
  PoseMap p = directStandPose();

  const double leftPhase = wrap01(walkTime / GAIT_CYCLE_TIME);
  const double rightPhase = wrap01(leftPhase + 0.5);
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
  const double lateral = HIP_ROLL_SHIFT * (leftSupport - rightSupport) * ROLL_TO_LEFT_DIR;
  p["left_hip_roll"] = lateral - SWING_HIP_ROLL_CENTER * left.swingWeight * ROLL_TO_LEFT_DIR;
  p["right_hip_roll"] = lateral + SWING_HIP_ROLL_CENTER * right.swingWeight * ROLL_TO_LEFT_DIR;
  p["left_ankle_roll"] = (-ANKLE_ROLL_SUPPORT * leftSupport + ANKLE_ROLL_PREP * rightSupport) * ROLL_TO_LEFT_DIR;
  p["right_ankle_roll"] = (ANKLE_ROLL_SUPPORT * rightSupport - ANKLE_ROLL_PREP * leftSupport) * ROLL_TO_LEFT_DIR;

  const double stepSignal = clamp((left.hip - right.hip) / (HEEL_STRIKE_HIP - TOE_OFF_HIP), -1.0, 1.0);
  const double swingEnergy = clamp(left.swingWeight + right.swingWeight, 0.0, 1.0);
  setArmSwing(p, -ARM_SWING * stepSignal, ARM_ROLL_LAND + 0.06 * swingEnergy, 0.16);
  p["left_shoulder_yaw"] = 0.12 * stepSignal;
  p["right_shoulder_yaw"] = 0.12 * stepSignal;
  p["waist_yaw"] = TORSO_YAW * stepSignal;
  p["neck_yaw"] = -0.35 * TORSO_YAW * stepSignal;
  p["waist_pitch"] = TORSO_FORWARD_LEAN + 0.025 * swingEnergy;

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

  setMotor("left_ankle_pitch", L_ANKLE_PITCH_SIGN * poseValue(p, "left_ankle_pitch"), -0.6, 0.6);
  setMotor("right_ankle_pitch", R_ANKLE_PITCH_SIGN * poseValue(p, "right_ankle_pitch"), -0.6, 0.6);

  setMotor("left_ankle_roll", L_ANKLE_ROLL_SIGN * poseValue(p, "left_ankle_roll"), -0.35, 0.35);
  setMotor("right_ankle_roll", R_ANKLE_ROLL_SIGN * poseValue(p, "right_ankle_roll"), -0.35, 0.35);
}
 
void applyWalkPose(Robot *robot) {
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
  applyTurnToPose(target);

  PoseMap desiredPose = mixPose(stand, target, startBlend);
  const double dt = previousApplyTime >= 0.0 ? t - previousApplyTime : 0.0;
  const double filterAlpha = dt > 0.0 ? 1.0 - std::exp(-dt / COMMAND_FILTER_TIME) : 1.0;
  PoseMap finalPose = filterPose(lastAppliedPose, desiredPose, filterAlpha);
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
    const double leftPhase = wrap01(walkTime / GAIT_CYCLE_TIME);
    const double rightPhase = wrap01(leftPhase + 0.5);

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

  std::cout << "joint steering enabled: DEFAULT_TURN_COMMAND=" << DEFAULT_TURN_COMMAND
            << " (positive turns left, negative turns right, 0 walks straight)" << std::endl;

  while (robot.step(timestep) != -1) {
    applyWalkPose(&robot);
    if (DEBUG_WALK)
      debugWalkState(&robot);
  }

  return 0;
}
