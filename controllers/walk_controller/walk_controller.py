from controller import Robot
import math


STEP_TIME = 1.4
HIP_AMP = 0.22
KNEE_AMP = 0.40
ANKLE_AMP = 0.18
ARM_AMP = 0.30
LATERAL_AMP = 0.025
WAIST_YAW_AMP = 0.04
PELVIS_BOUNCE_AMP = 0.008

BASE_HIP_PITCH = -0.06
BASE_KNEE_PITCH = 0.14
BASE_ANKLE_PITCH = -0.08

BASE_ELBOW = 0.45
BASE_SHOULDER_ROLL = 0.16

L_HIP_PITCH_SIGN = 1.0
R_HIP_PITCH_SIGN = 1.0
L_KNEE_SIGN = 1.0
R_KNEE_SIGN = 1.0
L_ANKLE_PITCH_SIGN = 1.0
R_ANKLE_PITCH_SIGN = 1.0

L_HIP_ROLL_SIGN = 1.0
R_HIP_ROLL_SIGN = 1.0
L_ANKLE_ROLL_SIGN = 1.0
R_ANKLE_ROLL_SIGN = 1.0

L_SHOULDER_PITCH_SIGN = 1.0
R_SHOULDER_PITCH_SIGN = 1.0

MOTOR_NAMES = [
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
    "right_ankle_roll",
]

LIMITS = {
    "neck_yaw": (-1.2, 1.2),

    "waist_yaw": (-0.6, 0.6),
    "waist_pitch": (-0.4, 0.4),

    "left_shoulder_pitch": (-1.8, 1.8),
    "right_shoulder_pitch": (-1.8, 1.8),
    "left_shoulder_roll": (-1.2, 1.2),
    "right_shoulder_roll": (-1.2, 1.2),
    "left_shoulder_yaw": (-1.5, 1.5),
    "right_shoulder_yaw": (-1.5, 1.5),
    "left_elbow_pitch": (0.0, 2.2),
    "right_elbow_pitch": (0.0, 2.2),

    "left_hip_roll": (-0.7, 0.7),
    "right_hip_roll": (-0.7, 0.7),
    "left_hip_pitch": (-1.2, 1.2),
    "right_hip_pitch": (-1.2, 1.2),
    "left_knee_pitch": (0.0, 2.2),
    "right_knee_pitch": (0.0, 2.2),
    "left_ankle_pitch": (-0.8, 0.8),
    "right_ankle_pitch": (-0.8, 0.8),
    "left_ankle_roll": (-0.5, 0.5),
    "right_ankle_roll": (-0.5, 0.5),
}

SIGN = {
    "left_hip_pitch": L_HIP_PITCH_SIGN,
    "right_hip_pitch": R_HIP_PITCH_SIGN,
    "left_knee_pitch": L_KNEE_SIGN,
    "right_knee_pitch": R_KNEE_SIGN,
    "left_ankle_pitch": L_ANKLE_PITCH_SIGN,
    "right_ankle_pitch": R_ANKLE_PITCH_SIGN,
    "left_hip_roll": L_HIP_ROLL_SIGN,
    "right_hip_roll": R_HIP_ROLL_SIGN,
    "left_ankle_roll": L_ANKLE_ROLL_SIGN,
    "right_ankle_roll": R_ANKLE_ROLL_SIGN,
    "left_shoulder_pitch": L_SHOULDER_PITCH_SIGN,
    "right_shoulder_pitch": R_SHOULDER_PITCH_SIGN,
}

SENSOR_NAMES = {}

STAND_POSE = {
    "neck_yaw": 0.0,

    "waist_yaw": 0.0,
    "waist_pitch": 0.02,

    "left_shoulder_pitch": 0.05,
    "right_shoulder_pitch": 0.05,
    "left_shoulder_roll": 0.16,
    "right_shoulder_roll": -0.16,
    "left_shoulder_yaw": 0.0,
    "right_shoulder_yaw": 0.0,
    "left_elbow_pitch": 0.45,
    "right_elbow_pitch": 0.45,

    "left_hip_roll": 0.02,
    "right_hip_roll": -0.02,
    "left_hip_pitch": -0.08,
    "right_hip_pitch": -0.08,
    "left_knee_pitch": 0.18,
    "right_knee_pitch": 0.18,
    "left_ankle_pitch": -0.10,
    "right_ankle_pitch": -0.10,
    "left_ankle_roll": 0.2,
    "right_ankle_roll": -0.2,
}

motors = {}


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def smooth_step(x):
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def set_motor(name, value, lo, hi):
    if name in motors:
        motors[name].setPosition(clamp(value, lo, hi))


def signed(name, value):
    return SIGN.get(name, 1.0) * value


def blend_pose(stand_pose, walk_pose, blend):
    pose = {}
    for name in MOTOR_NAMES:
        stand_angle = stand_pose.get(name, 0.0)
        walk_angle = walk_pose.get(name, stand_angle)
        pose[name] = stand_angle * (1.0 - blend) + walk_angle * blend
    return pose


def apply_pose(pose):
    for name, value in pose.items():
        lo, hi = LIMITS[name]
        set_motor(name, signed(name, value), lo, hi)


def walking_pose(t):
    walk_t = t - 2.0
    phase = 2.0 * math.pi * walk_t / STEP_TIME

    s = math.sin(phase)
    c = math.cos(phase)

    left_swing = max(0.0, s)
    right_swing = max(0.0, -s)
    swing = max(left_swing, right_swing)
    pelvis_bounce = PELVIS_BOUNCE_AMP * (0.5 - 0.5 * math.cos(2.0 * phase))

    left_hip_pitch = BASE_HIP_PITCH + HIP_AMP * s
    right_hip_pitch = BASE_HIP_PITCH - HIP_AMP * s

    left_knee_pitch = BASE_KNEE_PITCH + KNEE_AMP * left_swing + pelvis_bounce
    right_knee_pitch = BASE_KNEE_PITCH + KNEE_AMP * right_swing + pelvis_bounce

    left_ankle_pitch = BASE_ANKLE_PITCH - ANKLE_AMP * left_swing - 0.35 * left_hip_pitch - 0.6 * pelvis_bounce
    right_ankle_pitch = BASE_ANKLE_PITCH - ANKLE_AMP * right_swing - 0.35 * right_hip_pitch - 0.6 * pelvis_bounce

    left_hip_roll = 0.02 + LATERAL_AMP * c
    right_hip_roll = -0.02 + LATERAL_AMP * c

    left_shoulder_pitch = -ARM_AMP * s
    right_shoulder_pitch = ARM_AMP * s

    return {
        "neck_yaw": -0.03 * s,

        "waist_yaw": WAIST_YAW_AMP * s,
        "waist_pitch": 0.02 + 0.02 * swing,

        "left_shoulder_pitch": left_shoulder_pitch,
        "right_shoulder_pitch": right_shoulder_pitch,
        "left_shoulder_roll": BASE_SHOULDER_ROLL,
        "right_shoulder_roll": -BASE_SHOULDER_ROLL,
        "left_shoulder_yaw": 0.0,
        "right_shoulder_yaw": 0.0,
        "left_elbow_pitch": BASE_ELBOW + 0.05 * left_swing,
        "right_elbow_pitch": BASE_ELBOW + 0.05 * right_swing,

        "left_hip_roll": left_hip_roll,
        "right_hip_roll": right_hip_roll,
        "left_hip_pitch": left_hip_pitch,
        "right_hip_pitch": right_hip_pitch,
        "left_knee_pitch": left_knee_pitch,
        "right_knee_pitch": right_knee_pitch,
        "left_ankle_pitch": left_ankle_pitch,
        "right_ankle_pitch": right_ankle_pitch,
        "left_ankle_roll": 0.2,
        "right_ankle_roll": -0.2,
    }


def get_device(robot, name):
    try:
        return robot.getDevice(name)
    except Exception:
        return None


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    print("walk_controller started")

    for name in MOTOR_NAMES:
        motor = get_device(robot, name)
        if motor is None:
            print("WARNING missing motor: " + name)
            continue

        print("found motor: " + name)
        motors[name] = motor
        motor.setVelocity(6.0)
        try:
            motor.setAvailableTorque(100.0)
        except Exception:
            pass

        sensor = get_device(robot, SENSOR_NAMES.get(name, name + "_sensor"))
        if sensor is not None:
            try:
                sensor.enable(timestep)
            except Exception:
                pass

    apply_pose(STAND_POSE)

    while robot.step(timestep) != -1:
        t = robot.getTime()

        if t < 2.0:
            apply_pose(STAND_POSE)
            continue

        blend = smooth_step((t - 2.0) / 1.0)
        pose = blend_pose(STAND_POSE, walking_pose(t), blend)
        apply_pose(pose)


if __name__ == "__main__":
    main()
