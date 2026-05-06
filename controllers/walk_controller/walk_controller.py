from controller import Robot
import math


STEP_TIME = 2.02
STANCE_FRACTION = 0.72
SWING_FRACTION = 1.0 - STANCE_FRACTION
GAIT_START_PHASE = 0.5
COMMAND_FILTER_TIME = 0.22
STRIDE_HALF = 0.180
ARM_AMP = 0.36
LATERAL_AMP = 0.018
ANKLE_ROLL_SUPPORT = 0.004
WAIST_YAW_AMP = 0.0
PELVIS_BOUNCE_AMP = 0.008

BASE_HIP_PITCH = -0.06
BASE_KNEE_PITCH = 0.14
BASE_ANKLE_PITCH = -0.08

BASE_ELBOW = 0.62
BASE_SHOULDER_ROLL = 0.08

HEEL_STRIKE_HIP = 0.58
MID_STANCE_HIP = 0.02
TOE_OFF_HIP = -0.32
EARLY_SWING_HIP = 0.20
LATE_SWING_HIP = 0.74
LOADING_KNEE = 0.30
MID_STANCE_KNEE = 0.18
TOE_OFF_KNEE = 0.50
PEAK_SWING_KNEE = 1.02
SWING_RECOVERY_KNEE = 0.30
HEEL_STRIKE_ANKLE = 0.0
FOOT_FLAT_ANKLE = 0.00
MID_STANCE_ANKLE = 0.0
TOE_OFF_ANKLE = 0.0
SWING_TOE_UP_ANKLE = 0.40
SWING_CLEARANCE_ANKLE = 0.32
ANKLE_PITCH_LIMIT = 0.85
IK_BLEND = 0.35
IK_THIGH_LENGTH = 0.18
IK_SHIN_LENGTH = 0.32
IK_STANCE_ANKLE_DOWN = 0.47
IK_SWING_FOOT_LIFT = 0.030
IK_SWING_FOOT_PITCH = 0.09
GROUND_FOOT_PITCH = 0.0
SWING_FOOT_TOE_UP_PITCH = 0.03
SWING_FOOT_CLEARANCE_PITCH = -0.26
SWING_FOOT_LANDING_PITCH = -0.22
SWING_ANKLE_DOWN_EARLY = -0.42
SWING_ANKLE_DOWN_MID = -0.58
SWING_ANKLE_DOWN_LANDING = -0.38
IK_NEUTRAL_HIP = 0.02
IK_NEUTRAL_KNEE = MID_STANCE_KNEE
IK_NEUTRAL_ANKLE = 0.0
BALANCE_ROLL_GAIN = 0.08
BALANCE_RATE_GAIN = 0.004
BALANCE_MAX = 0.006
HIP_ROLL_LIMIT = 0.028
ANKLE_ROLL_LIMIT = 0.012

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
    "left_ankle_pitch": (-ANKLE_PITCH_LIMIT, ANKLE_PITCH_LIMIT),
    "right_ankle_pitch": (-ANKLE_PITCH_LIMIT, ANKLE_PITCH_LIMIT),
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
    "waist_pitch": 0.0,

    "left_shoulder_pitch": 0.05,
    "right_shoulder_pitch": 0.05,
    "left_shoulder_roll": 0.08,
    "right_shoulder_roll": -0.08,
    "left_shoulder_yaw": 0.0,
    "right_shoulder_yaw": 0.0,
    "left_elbow_pitch": 0.50,
    "right_elbow_pitch": 0.50,

    "left_hip_roll": 0.0,
    "right_hip_roll": 0.0,
    "left_hip_pitch": -0.04,
    "right_hip_pitch": -0.04,
    "left_knee_pitch": 0.08,
    "right_knee_pitch": 0.08,
    "left_ankle_pitch": 0.12,
    "right_ankle_pitch": 0.12,
    "left_ankle_roll": 0.0,
    "right_ankle_roll": 0.0,
}

motors = {}


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def smooth_step(x):
    x = clamp(x, 0.0, 1.0)
    return x * x * x * (x * (x * 6.0 - 15.0) + 10.0)


def rolling_ease(x):
    x = clamp(x, 0.0, 1.0)
    return 0.55 * x + 0.45 * (0.5 - 0.5 * math.cos(math.pi * x))


def wrap01(x):
    x = math.fmod(x, 1.0)
    if x < 0.0:
        x += 1.0
    return x


def left_phase_for_walk_time(walk_t):
    return wrap01(walk_t / STEP_TIME + GAIT_START_PHASE)


def right_phase_for_walk_time(walk_t):
    return wrap01(left_phase_for_walk_time(walk_t) + 0.5)


def soft_bell(x):
    return math.sin(math.pi * smooth_step(x))


def swing_weight_for_phase(phase):
    phase = wrap01(phase)
    if phase < STANCE_FRACTION:
        return 0.0
    swing_phase = (phase - STANCE_FRACTION) / SWING_FRACTION
    return soft_bell(swing_phase)


def support_weight_for_phase(phase):
    phase = wrap01(phase)
    if phase >= STANCE_FRACTION:
        return 0.0
    stance_phase = phase / STANCE_FRACTION
    return smooth_step(stance_phase / 0.26) * (1.0 - smooth_step((stance_phase - 0.74) / 0.26))


def foot_x_for_phase(phase):
    phase = wrap01(phase)
    if phase < STANCE_FRACTION:
        return STRIDE_HALF + (-2.0 * STRIDE_HALF) * rolling_ease(phase / STANCE_FRACTION)
    swing_phase = (phase - STANCE_FRACTION) / SWING_FRACTION
    return -STRIDE_HALF + (2.0 * STRIDE_HALF) * rolling_ease((swing_phase - 0.10) / 0.82)


def foot_target_for_phase(phase):
    phase = wrap01(phase)
    foot_x = foot_x_for_phase(phase)
    swing_weight = swing_weight_for_phase(phase)
    max_reach = IK_THIGH_LENGTH + IK_SHIN_LENGTH - 0.004
    reach_limited_down = math.sqrt(max(0.001, max_reach * max_reach - foot_x * foot_x))
    z_down = min(IK_STANCE_ANKLE_DOWN, reach_limited_down) - IK_SWING_FOOT_LIFT * swing_weight
    return {
        "x": foot_x,
        "z_down": z_down,
        "pitch": IK_SWING_FOOT_PITCH * swing_weight,
        "swing": swing_weight,
        "support": support_weight_for_phase(phase),
    }


def solve_raw_sagittal_ik(x, z_down, foot_pitch):
    min_reach = abs(IK_SHIN_LENGTH - IK_THIGH_LENGTH) + 0.004
    max_reach = IK_THIGH_LENGTH + IK_SHIN_LENGTH - 0.004
    reach = clamp(math.hypot(x, z_down), min_reach, max_reach)
    cos_knee = clamp(
        (reach * reach - IK_THIGH_LENGTH * IK_THIGH_LENGTH - IK_SHIN_LENGTH * IK_SHIN_LENGTH)
        / (2.0 * IK_THIGH_LENGTH * IK_SHIN_LENGTH),
        -1.0,
        1.0,
    )
    knee = math.acos(cos_knee)
    hip = math.atan2(x, z_down) - math.atan2(
        IK_SHIN_LENGTH * math.sin(knee),
        IK_THIGH_LENGTH + IK_SHIN_LENGTH * math.cos(knee),
    )
    # Knee pitch uses the opposite joint axis from hip/ankle pitch in the PROTO,
    # so world foot pitch is hip - knee + ankle.
    ankle = foot_pitch - hip + knee
    return hip, knee, ankle


def ground_locked_ankle_for_pose(hip, knee, foot_pitch=GROUND_FOOT_PITCH):
    return clamp(foot_pitch - hip + knee, -ANKLE_PITCH_LIMIT, ANKLE_PITCH_LIMIT)


def swing_foot_pitch_for_phase(phase):
    phase = wrap01(phase)
    if phase < STANCE_FRACTION:
        return GROUND_FOOT_PITCH
    swing_phase = (phase - STANCE_FRACTION) / SWING_FRACTION
    if swing_phase < 0.24:
        u = rolling_ease(swing_phase / 0.24)
        return GROUND_FOOT_PITCH + (SWING_FOOT_TOE_UP_PITCH - GROUND_FOOT_PITCH) * u
    if swing_phase < 0.62:
        u = rolling_ease((swing_phase - 0.24) / 0.38)
        return SWING_FOOT_TOE_UP_PITCH + (SWING_FOOT_CLEARANCE_PITCH - SWING_FOOT_TOE_UP_PITCH) * u
    u = rolling_ease((swing_phase - 0.62) / 0.38)
    return SWING_FOOT_CLEARANCE_PITCH + (SWING_FOOT_LANDING_PITCH - SWING_FOOT_CLEARANCE_PITCH) * u


def swing_ankle_pitch_for_phase(phase):
    swing_phase = (wrap01(phase) - STANCE_FRACTION) / SWING_FRACTION
    if swing_phase < 0.20:
        u = rolling_ease(swing_phase / 0.20)
        return TOE_OFF_ANKLE + (SWING_ANKLE_DOWN_EARLY - TOE_OFF_ANKLE) * u
    if swing_phase < 0.62:
        u = rolling_ease((swing_phase - 0.20) / 0.42)
        return SWING_ANKLE_DOWN_EARLY + (SWING_ANKLE_DOWN_MID - SWING_ANKLE_DOWN_EARLY) * u
    u = rolling_ease((swing_phase - 0.62) / 0.38)
    return SWING_ANKLE_DOWN_MID + (SWING_ANKLE_DOWN_LANDING - SWING_ANKLE_DOWN_MID) * u


def curve_leg_angles_for_phase(phase):
    phase = wrap01(phase)
    target = foot_target_for_phase(phase)
    foot_x = target["x"]
    swing_weight = target["swing"]

    if phase < STANCE_FRACTION:
        stance_phase = phase / STANCE_FRACTION
        if stance_phase < 0.18:
            u = rolling_ease(stance_phase / 0.18)
            hip = HEEL_STRIKE_HIP + (0.38 - HEEL_STRIKE_HIP) * u
            knee = SWING_RECOVERY_KNEE + (LOADING_KNEE - SWING_RECOVERY_KNEE) * u
            ankle = HEEL_STRIKE_ANKLE + (FOOT_FLAT_ANKLE - HEEL_STRIKE_ANKLE) * u
        elif stance_phase < 0.62:
            u = rolling_ease((stance_phase - 0.18) / 0.44)
            hip = 0.38 + (MID_STANCE_HIP - 0.38) * u
            knee = LOADING_KNEE + (MID_STANCE_KNEE - LOADING_KNEE) * u
            ankle = FOOT_FLAT_ANKLE + (MID_STANCE_ANKLE - FOOT_FLAT_ANKLE) * u
        elif stance_phase < 0.84:
            u = rolling_ease((stance_phase - 0.62) / 0.22)
            hip = MID_STANCE_HIP + (-0.22 - MID_STANCE_HIP) * u
            knee = MID_STANCE_KNEE + (TOE_OFF_KNEE - MID_STANCE_KNEE) * u
            ankle = MID_STANCE_ANKLE + (TOE_OFF_ANKLE - MID_STANCE_ANKLE) * u
        else:
            u = rolling_ease((stance_phase - 0.84) / 0.16)
            hip = -0.22 + (TOE_OFF_HIP + 0.22) * u
            knee = TOE_OFF_KNEE + (0.62 - TOE_OFF_KNEE) * u
            ankle = TOE_OFF_ANKLE
    else:
        swing_phase = (phase - STANCE_FRACTION) / SWING_FRACTION
        if swing_phase < 0.24:
            u = rolling_ease(swing_phase / 0.24)
            hip = TOE_OFF_HIP + (EARLY_SWING_HIP - TOE_OFF_HIP) * u
            knee = 0.62 + (PEAK_SWING_KNEE - 0.62) * u
            ankle = TOE_OFF_ANKLE + (SWING_TOE_UP_ANKLE - TOE_OFF_ANKLE) * u
        elif swing_phase < 0.62:
            u = rolling_ease((swing_phase - 0.24) / 0.38)
            hip = EARLY_SWING_HIP + (LATE_SWING_HIP - EARLY_SWING_HIP) * u
            knee = PEAK_SWING_KNEE + (0.70 - PEAK_SWING_KNEE) * u
            ankle = SWING_TOE_UP_ANKLE + (SWING_CLEARANCE_ANKLE - SWING_TOE_UP_ANKLE) * u
        else:
            u = rolling_ease((swing_phase - 0.62) / 0.38)
            hip = LATE_SWING_HIP + (HEEL_STRIKE_HIP - LATE_SWING_HIP) * u
            knee = 0.70 + (SWING_RECOVERY_KNEE - 0.70) * u
            ankle = SWING_CLEARANCE_ANKLE + (HEEL_STRIKE_ANKLE - SWING_CLEARANCE_ANKLE) * u

    return {
        "hip": clamp(hip, -1.2, 1.2),
        "knee": clamp(knee, 0.0, 2.2),
        "ankle": clamp(ankle, -ANKLE_PITCH_LIMIT, ANKLE_PITCH_LIMIT),
        "swing": swing_weight,
        "support": target["support"],
        "foot_x": foot_x,
    }


def ik_leg_angles_for_phase(phase):
    target = foot_target_for_phase(phase)
    neutral_hip, neutral_knee, neutral_ankle = solve_raw_sagittal_ik(0.0, IK_STANCE_ANKLE_DOWN, 0.0)
    raw_hip, raw_knee, raw_ankle = solve_raw_sagittal_ik(target["x"], target["z_down"], target["pitch"])
    return {
        "hip": clamp(IK_NEUTRAL_HIP + (raw_hip - neutral_hip), -1.2, 1.2),
        "knee": clamp(IK_NEUTRAL_KNEE + (raw_knee - neutral_knee), 0.0, 2.2),
        "ankle": clamp(IK_NEUTRAL_ANKLE + (raw_ankle - neutral_ankle), -ANKLE_PITCH_LIMIT, ANKLE_PITCH_LIMIT),
        "swing": target["swing"],
        "support": target["support"],
        "foot_x": target["x"],
    }


def leg_angles_for_phase(phase):
    curve = curve_leg_angles_for_phase(phase)
    ik = ik_leg_angles_for_phase(phase)
    hip = curve["hip"] + (ik["hip"] - curve["hip"]) * IK_BLEND
    knee = curve["knee"] + (ik["knee"] - curve["knee"]) * IK_BLEND
    ankle = curve["ankle"] + (ik["ankle"] - curve["ankle"]) * IK_BLEND
    if wrap01(phase) < STANCE_FRACTION:
        ankle = ground_locked_ankle_for_pose(hip, knee, GROUND_FOOT_PITCH)
    else:
        ankle = clamp(swing_ankle_pitch_for_phase(phase), -ANKLE_PITCH_LIMIT, ANKLE_PITCH_LIMIT)
    return {
        "hip": hip,
        "knee": knee,
        "ankle": ankle,
        "swing": curve["swing"],
        "support": curve["support"],
        "foot_x": curve["foot_x"],
    }


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


def filter_pose(previous_pose, target_pose, alpha):
    if previous_pose is None:
        return dict(target_pose)

    alpha = clamp(alpha, 0.0, 1.0)
    pose = {}
    for name in MOTOR_NAMES:
        previous_angle = previous_pose.get(name, target_pose.get(name, 0.0))
        target_angle = target_pose.get(name, previous_angle)
        pose[name] = previous_angle + (target_angle - previous_angle) * alpha
    return pose


def apply_pose(pose):
    for name, value in pose.items():
        lo, hi = LIMITS[name]
        set_motor(name, signed(name, value), lo, hi)


def imu_balance_correction(imu, t, previous_roll, previous_time):
    if imu is None:
        return 0.0, previous_roll, previous_time

    try:
        roll = imu.getRollPitchYaw()[0]
    except Exception:
        return 0.0, previous_roll, previous_time

    if previous_roll is None or previous_time is None:
        return 0.0, roll, t

    dt = t - previous_time
    roll_rate = (roll - previous_roll) / dt if dt > 0.0 else 0.0
    correction = clamp(
        -(BALANCE_ROLL_GAIN * roll + BALANCE_RATE_GAIN * roll_rate),
        -BALANCE_MAX,
        BALANCE_MAX,
    )
    return correction, roll, t


def walking_pose(t, balance_roll=0.0):
    walk_t = t - 2.0
    left_phase = left_phase_for_walk_time(walk_t)
    right_phase = right_phase_for_walk_time(walk_t)
    left = leg_angles_for_phase(left_phase)
    right = leg_angles_for_phase(right_phase)
    step_signal = clamp((left["foot_x"] - right["foot_x"]) / (2.0 * STRIDE_HALF), -1.0, 1.0)
    swing = clamp(left["swing"] + right["swing"], 0.0, 1.0)
    support_balance = clamp(left["support"] - right["support"], -1.0, 1.0)

    left_hip_pitch = left["hip"]
    right_hip_pitch = right["hip"]
    left_knee_pitch = left["knee"]
    right_knee_pitch = right["knee"]
    left_ankle_pitch = left["ankle"]
    right_ankle_pitch = right["ankle"]

    left_hip_roll = clamp(LATERAL_AMP * support_balance + balance_roll, -HIP_ROLL_LIMIT, HIP_ROLL_LIMIT)
    right_hip_roll = clamp(LATERAL_AMP * support_balance + balance_roll, -HIP_ROLL_LIMIT, HIP_ROLL_LIMIT)
    ankle_roll = clamp(
        -ANKLE_ROLL_SUPPORT * support_balance - 0.75 * balance_roll,
        -ANKLE_ROLL_LIMIT,
        ANKLE_ROLL_LIMIT,
    )

    arm_signal = math.cos(2.0 * math.pi * right_phase)
    torso_twist = WAIST_YAW_AMP * step_signal
    left_shoulder_pitch = ARM_AMP * arm_signal
    right_shoulder_pitch = -ARM_AMP * arm_signal
    effort = clamp(abs(left_shoulder_pitch) / ARM_AMP, 0.0, 1.0)
    forward_elbow = clamp(BASE_ELBOW - 0.05 * effort, 0.32, 2.2)
    backward_elbow = clamp(BASE_ELBOW + 0.08 * effort, 0.0, 2.2)

    return {
        "neck_yaw": 0.0,

        "waist_yaw": torso_twist,
        "waist_pitch": clamp(-0.062 - 0.006 * swing, -0.10, -0.025),

        "left_shoulder_pitch": left_shoulder_pitch,
        "right_shoulder_pitch": right_shoulder_pitch,
        "left_shoulder_roll": BASE_SHOULDER_ROLL,
        "right_shoulder_roll": -BASE_SHOULDER_ROLL,
        "left_shoulder_yaw": 0.0,
        "right_shoulder_yaw": 0.0,
        "left_elbow_pitch": forward_elbow if left_shoulder_pitch > 0.0 else backward_elbow,
        "right_elbow_pitch": forward_elbow if right_shoulder_pitch > 0.0 else backward_elbow,

        "left_hip_roll": left_hip_roll,
        "right_hip_roll": right_hip_roll,
        "left_hip_pitch": left_hip_pitch,
        "right_hip_pitch": right_hip_pitch,
        "left_knee_pitch": left_knee_pitch,
        "right_knee_pitch": right_knee_pitch,
        "left_ankle_pitch": left_ankle_pitch,
        "right_ankle_pitch": right_ankle_pitch,
        "left_ankle_roll": ankle_roll,
        "right_ankle_roll": ankle_roll,
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
        motor.setVelocity(8.0 if "ankle" in name else 6.0)
        try:
            motor.setAvailableTorque(80.0 if "ankle" in name else 100.0)
        except Exception:
            pass

        sensor = get_device(robot, SENSOR_NAMES.get(name, name + "_sensor"))
        if sensor is not None:
            try:
                sensor.enable(timestep)
            except Exception:
                pass

    apply_pose(STAND_POSE)
    last_pose = None
    previous_time = None
    previous_roll = None
    previous_balance_time = None

    body_imu = get_device(robot, "body_imu")
    if body_imu is not None:
        try:
            body_imu.enable(timestep)
        except Exception:
            body_imu = None

    while robot.step(timestep) != -1:
        t = robot.getTime()

        if t < 2.0:
            _, previous_roll, previous_balance_time = imu_balance_correction(
                body_imu, t, previous_roll, previous_balance_time
            )
            apply_pose(STAND_POSE)
            last_pose = dict(STAND_POSE)
            previous_time = t
            continue

        blend = smooth_step((t - 2.0) / 1.0)
        balance_roll, previous_roll, previous_balance_time = imu_balance_correction(
            body_imu, t, previous_roll, previous_balance_time
        )
        pose = blend_pose(STAND_POSE, walking_pose(t, balance_roll), blend)
        dt = t - previous_time if previous_time is not None else 0.0
        alpha = 1.0 - math.exp(-dt / COMMAND_FILTER_TIME) if dt > 0.0 else 1.0
        pose = filter_pose(last_pose, pose, alpha)
        apply_pose(pose)
        last_pose = pose
        previous_time = t


if __name__ == "__main__":
    main()
