#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
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

const double HIP_FORWARD_DIR = 1.0;
const double ROLL_TO_LEFT_DIR = 1.0;

const double SHIFT_TIME = 0.23;
const double SWING_TIME = 0.31;
const double DOWN_TIME = 0.15;
const bool RESCUE_STAND_ONLY = false;
const bool DEBUG_WALK = false;

const double ARM_SWING = 0.48;
const double ARM_SWING_LAND = 0.34;
const double ARM_ROLL_SWING = 0.18;
const double ARM_ROLL_LAND = 0.15;
const double ARM_ELBOW_BASE = 0.52;
const double ARM_ELBOW_BACK_GAIN = 0.04;
const double TORSO_YAW = 0.015;
const double TORSO_PITCH_SHIFT = 0.012;
const double TORSO_PITCH_WALK = 0.026;

const double HIP_ROLL_SHIFT = 0.08;
const double ANKLE_ROLL_SUPPORT = 0.07;
const double ANKLE_ROLL_PREP = 0.025;
const double SUPPORT_HIP_BACK = -0.22;
const double SWING_HIP_FORWARD = 0.40;
const double LAND_HIP_BACK = -0.16;
const double LAND_HIP_FORWARD = 0.26;
const double SUPPORT_KNEE = 0.14;
const double SWING_PREP_KNEE = 0.18;
const double SWING_KNEE = 0.58;
const double LAND_KNEE = 0.20;
const double SUPPORT_ANKLE_PUSH = 0.08;
const double LAND_SUPPORT_ANKLE = 0.06;
const double SWING_ANKLE_LIFT = -0.22;
const double LAND_ANKLE = -0.08;

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
    handle.motor->setVelocity(3.1);
    handle.motor->setAvailableTorque(60.0);

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

  p["left_hip_pitch"] = -0.10 * HIP_FORWARD_DIR;
  p["right_hip_pitch"] = -0.04 * HIP_FORWARD_DIR;
  p["left_knee_pitch"] = SUPPORT_KNEE;
  p["right_knee_pitch"] = SWING_PREP_KNEE;
  p["left_ankle_pitch"] = LAND_SUPPORT_ANKLE * HIP_FORWARD_DIR;
  p["right_ankle_pitch"] = LAND_ANKLE * HIP_FORWARD_DIR;

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

  p["left_hip_pitch"] = -0.04 * HIP_FORWARD_DIR;
  p["right_hip_pitch"] = -0.10 * HIP_FORWARD_DIR;
  p["left_knee_pitch"] = SWING_PREP_KNEE;
  p["right_knee_pitch"] = SUPPORT_KNEE;
  p["left_ankle_pitch"] = LAND_ANKLE * HIP_FORWARD_DIR;
  p["right_ankle_pitch"] = LAND_SUPPORT_ANKLE * HIP_FORWARD_DIR;

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

  setMotor("left_ankle_pitch", L_ANKLE_PITCH_SIGN * poseValue(p, "left_ankle_pitch"), -0.35, 0.35);
  setMotor("right_ankle_pitch", R_ANKLE_PITCH_SIGN * poseValue(p, "right_ankle_pitch"), -0.35, 0.35);

  setMotor("left_ankle_roll", L_ANKLE_ROLL_SIGN * poseValue(p, "left_ankle_roll"), -0.20, 0.20);
  setMotor("right_ankle_roll", R_ANKLE_ROLL_SIGN * poseValue(p, "right_ankle_roll"), -0.20, 0.20);
}

void applyWalkPose(Robot *robot) {
  double t = robot->getTime();

  if (RESCUE_STAND_ONLY || t < 2.0) {
    PoseMap standPose = directStandPose();
    lastAppliedPose = standPose;
    applyDirectPose(standPose);
    return;
  }

  double startBlend = smoothStep(clamp((t - 2.0) / 1.0, 0.0, 1.0));

  const double cycleTime = 2.0 * (SHIFT_TIME + SWING_TIME + DOWN_TIME);

  double wt = std::fmod(t - 2.0, cycleTime);
  if (wt < 0.0)
    wt += cycleTime;

  PoseMap stand = directStandPose();
  PoseMap target = stand;

  double s0 = SHIFT_TIME;
  double s1 = s0 + SWING_TIME;
  double s2 = s1 + DOWN_TIME;
  double s3 = s2 + SHIFT_TIME;
  double s4 = s3 + SWING_TIME;
  double s5 = s4 + DOWN_TIME;

  if (wt < s0) {
    double u = wt / SHIFT_TIME;
    target = mixPose(leftFootDownPose(), shiftLeftPose(), u);
  } else if (wt < s1) {
    double u = (wt - s0) / SWING_TIME;
    target = mixPose(shiftLeftPose(), swingRightPose(), u);
  } else if (wt < s2) {
    double u = (wt - s1) / DOWN_TIME;
    target = mixPose(swingRightPose(), rightFootDownPose(), u);
  } else if (wt < s3) {
    double u = (wt - s2) / SHIFT_TIME;
    target = mixPose(rightFootDownPose(), shiftRightPose(), u);
  } else if (wt < s4) {
    double u = (wt - s3) / SWING_TIME;
    target = mixPose(shiftRightPose(), swingLeftPose(), u);
  } else if (wt < s5) {
    double u = (wt - s4) / DOWN_TIME;
    target = mixPose(swingLeftPose(), leftFootDownPose(), u);
  }

  PoseMap finalPose = mixPose(stand, target, startBlend);
  lastAppliedPose = finalPose;
  applyDirectPose(finalPose);
}

void debugWalkState(Supervisor *robot) {
  static double lastPrintTime = -1.0;
  static bool havePreviousRoot = false;
  static double previousRoot[3] = {0.0, 0.0, 0.0};
  double t = robot->getTime();

  if (lastPrintTime >= 0.0 && t - lastPrintTime < 0.2)
    return;
  lastPrintTime = t;

  Node *self = robot->getSelf();

  const double *rootPos = self ? self->getPosition() : nullptr;
  const double *leftFootPos = leftFootGps ? leftFootGps->getValues() : nullptr;
  const double *rightFootPos = rightFootGps ? rightFootGps->getValues() : nullptr;

  Node *leftFoot = nullptr;
  Node *rightFoot = nullptr;
  if (!leftFootPos) {
    leftFoot = robot->getFromDef("LEFT_FOOT");
    leftFootPos = leftFoot ? leftFoot->getPosition() : nullptr;
  }
  if (!rightFootPos) {
    rightFoot = robot->getFromDef("RIGHT_FOOT");
    rightFootPos = rightFoot ? rightFoot->getPosition() : nullptr;
  }
  const double *rpy = bodyImu ? bodyImu->getRollPitchYaw() : nullptr;

  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;
  if (rootPos && havePreviousRoot) {
    dx = rootPos[0] - previousRoot[0];
    dy = rootPos[1] - previousRoot[1];
    dz = rootPos[2] - previousRoot[2];
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
    const double cycleTime = 2.0 * (SHIFT_TIME + SWING_TIME + DOWN_TIME);
    double wt = std::fmod(t - 2.0, cycleTime);
    if (wt < 0.0)
      wt += cycleTime;

    double s0 = SHIFT_TIME;
    double s1 = s0 + SWING_TIME;
    double s2 = s1 + DOWN_TIME;
    double s3 = s2 + SHIFT_TIME;
    double s4 = s3 + SWING_TIME;

    if (wt < s0) {
      supportFoot = "left";
      swingFoot = "right-prep";
    } else if (wt < s1) {
      supportFoot = "left";
      swingFoot = "right";
    } else if (wt < s2) {
      supportFoot = "left";
      swingFoot = "right-down";
    } else if (wt < s3) {
      supportFoot = "right";
      swingFoot = "left-prep";
    } else if (wt < s4) {
      supportFoot = "right";
      swingFoot = "left";
    } else {
      supportFoot = "right";
      swingFoot = "left-down";
    }
  }

  std::cout << std::fixed << std::setprecision(3)
            << "[walk-debug] t=" << t;
  if (rootPos) {
    std::cout << " root=(" << rootPos[0] << ", " << rootPos[1] << ", " << rootPos[2] << ")"
              << " dRoot=(" << dx << ", " << dy << ", " << dz << ")";
  } else {
    std::cout << " root=(unavailable)";
  }
  if (leftFootPos) {
    std::cout << " leftFoot=(" << leftFootPos[0] << ", " << leftFootPos[1] << ", " << leftFootPos[2] << ")";
  } else {
    std::cout << " leftFoot=(unavailable)";
  }
  if (rightFootPos) {
    std::cout << " rightFoot=(" << rightFootPos[0] << ", " << rightFootPos[1] << ", " << rightFootPos[2] << ")";
  } else {
    std::cout << " rightFoot=(unavailable)";
  }
  if (rpy) {
    std::cout << " imu=(" << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << ")";
  } else {
    std::cout << " imu=(unavailable)";
  }
  std::cout << " support=" << supportFoot
            << " swing=" << swingFoot
            << " L(h/k/a)=" << poseValue(lastAppliedPose, "left_hip_pitch") << "/"
            << poseValue(lastAppliedPose, "left_knee_pitch") << "/"
            << poseValue(lastAppliedPose, "left_ankle_pitch")
            << " R(h/k/a)=" << poseValue(lastAppliedPose, "right_hip_pitch") << "/"
            << poseValue(lastAppliedPose, "right_knee_pitch") << "/"
            << poseValue(lastAppliedPose, "right_ankle_pitch")
            << std::endl;
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

  if (DEBUG_WALK) {
    leftFootGps = robot.getGPS("left_foot_gps");
    if (leftFootGps) {
      leftFootGps->enable(timestep);
      std::cout << "found sensor: left_foot_gps" << std::endl;
    } else {
      std::cout << "WARNING missing sensor: left_foot_gps" << std::endl;
    }

    rightFootGps = robot.getGPS("right_foot_gps");
    if (rightFootGps) {
      rightFootGps->enable(timestep);
      std::cout << "found sensor: right_foot_gps" << std::endl;
    } else {
      std::cout << "WARNING missing sensor: right_foot_gps" << std::endl;
    }

    bodyImu = robot.getInertialUnit("body_imu");
    if (bodyImu) {
      bodyImu->enable(timestep);
      std::cout << "found sensor: body_imu" << std::endl;
    } else {
      std::cout << "WARNING missing sensor: body_imu" << std::endl;
    }
  }

  while (robot.step(timestep) != -1) {
    applyWalkPose(&robot);
    if (DEBUG_WALK)
      debugWalkState(&robot);
  }

  return 0;
}
