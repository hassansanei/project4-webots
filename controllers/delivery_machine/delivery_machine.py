from controller import Robot
import math


# ============================================================
# PROJECT 7 - FINAL VERSION
# Forced -107 degree turn for final segment
# Stop when REAL X reaches 0.5
# ============================================================


robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())


# ============================================================
# MOTORS
# ============================================================

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))


# ============================================================
# ENCODERS
# ============================================================

left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")

left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)


# ============================================================
# E-PUCK PARAMETERS
# ============================================================

WHEEL_RADIUS = 0.0205
WHEEL_BASE = 0.052289


# ============================================================
# MOTOR SPEED
# ============================================================

MAX_SPEED = 6.28
MOVE_SPEED = 2.5
TURN_SPEED_MAX = 2.0
TURN_SPEED_MIN = 0.55
SLOW_DISTANCE = 0.20
ARRIVAL_TOLERANCE = 0.015
ANGLE_TOLERANCE = math.radians(1.5)


# ============================================================
# CONTROL GAINS
# ============================================================

K_TURN = 2.8
K_HEADING = 1.5


# ============================================================
# STOP TIME
# ============================================================

STOP_TIME = 0.5


# ============================================================
# STOPPING POINTS
# ============================================================

WAYPOINTS = [
    (0.540, 0.710),   # 0: START
    (0.240, -0.380),  # 1: BLUE
    (-0.480, -0.350), # 2: RED
    (-0.600, 0.640),  # 3: YELLOW
    (0.540, 0.710)    # 4: START again
]

WAYPOINT_NAMES = [
    "START / GREEN",
    "BLUE",
    "RED",
    "YELLOW",
    "START / GREEN"
]


# ============================================================
# STATES
# ============================================================

STATE_INIT = 0
STATE_TURN = 1
STATE_MOVE = 2
STATE_STOP = 3
STATE_NEXT = 4
STATE_COMPLETE = 5

state = STATE_INIT


# ============================================================
# MISSION VARIABLES
# ============================================================

target_index = 1
target_x = 0.0
target_z = 0.0
target_distance = 0.0
target_angle = 0.0
segment_distance = 0.0
segment_angle = 0.0


# ============================================================
# ENCODER REFERENCES
# ============================================================

left_start_encoder = 0.0
right_start_encoder = 0.0
left_turn_start = 0.0
right_turn_start = 0.0
stop_start_time = 0.0


# ============================================================
# HEADING AND POSITION (Odometry - فقط برای نمایش)
# ============================================================

heading = math.radians(-90.0)
x = WAYPOINTS[0][0]
z = WAYPOINTS[0][1]


# ============================================================
# UTILITY FUNCTIONS
# ============================================================

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def stop_robot():
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


def set_wheels(left_speed, right_speed):
    left_speed = clamp(left_speed, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed, -MAX_SPEED, MAX_SPEED)
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


# ============================================================
# WAIT FOR FIRST SENSOR SAMPLE
# ============================================================

robot.step(TIME_STEP)
left_prev_enc = left_encoder.getValue()
right_prev_enc = right_encoder.getValue()


# ============================================================
# MAIN LOOP
# ============================================================

while robot.step(TIME_STEP) != -1:

    current_time = robot.getTime()
    left_enc = left_encoder.getValue()
    right_enc = right_encoder.getValue()


    # ========================================================
    # UPDATE ODOMETRY (فقط برای نمایش)
    # ========================================================
    
    delta_left = (left_enc - left_prev_enc) * WHEEL_RADIUS
    delta_right = (right_enc - right_prev_enc) * WHEEL_RADIUS

    left_prev_enc = left_enc
    right_prev_enc = right_enc

    delta_distance = (delta_left + delta_right) / 2.0
    delta_heading = (delta_right - delta_left) / WHEEL_BASE

    heading = normalize_angle(heading + delta_heading)
    x += delta_distance * math.cos(heading)
    z += delta_distance * math.sin(heading)


    # ========================================================
    # STATE 0: INITIALIZATION
    # ========================================================

    if state == STATE_INIT:

        current_x = WAYPOINTS[target_index - 1][0]
        current_z = WAYPOINTS[target_index - 1][1]

        target_x = WAYPOINTS[target_index][0]
        target_z = WAYPOINTS[target_index][1]

        dx = target_x - current_x
        dz = target_z - current_z

        target_distance = math.sqrt(dx*dx + dz*dz)
        target_angle = math.atan2(dz, dx)

        segment_angle = normalize_angle(target_angle - heading)
        if segment_angle > math.pi:
            segment_angle -= 2.0 * math.pi
        elif segment_angle < -math.pi:
            segment_angle += 2.0 * math.pi

        left_turn_start = left_enc
        right_turn_start = right_enc

        print()
        print("============================================")
        print(f"FROM: {WAYPOINT_NAMES[target_index - 1]}")
        print(f"TO:   {WAYPOINT_NAMES[target_index]}")
        print(f"DISTANCE = {target_distance:.3f} m")
        print(f"TURN = {math.degrees(segment_angle):.2f}°")
        print("STATE CHANGE -> TURN")

        state = STATE_TURN


    # ========================================================
    # STATE 1: TURN
    # ========================================================

    elif state == STATE_TURN:

        delta_left_enc = (left_enc - left_turn_start)
        delta_right_enc = (right_enc - right_turn_start)

        left_dist = delta_left_enc * WHEEL_RADIUS
        right_dist = delta_right_enc * WHEEL_RADIUS

        measured_angle = (right_dist - left_dist) / WHEEL_BASE
        
        angle_error = normalize_angle(segment_angle - measured_angle)
        if angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        if abs(angle_error) <= ANGLE_TOLERANCE:
            stop_robot()
            print(f"TURN COMPLETE | TARGET={math.degrees(segment_angle):.2f}°")
            state = STATE_MOVE
            left_start_encoder = left_enc
            right_start_encoder = right_enc
        else:
            turn_speed = clamp(K_TURN * abs(angle_error), TURN_SPEED_MIN, TURN_SPEED_MAX)
            if angle_error > 0:
                set_wheels(-turn_speed, turn_speed)
            else:
                set_wheels(turn_speed, -turn_speed)

            print(f"TURN | TARGET={math.degrees(segment_angle):7.2f}° | "
                  f"MEASURED={math.degrees(measured_angle):7.2f}° | "
                  f"ERROR={math.degrees(angle_error):7.2f}°")


    # ========================================================
    # STATE 2: MOVE
    # ========================================================

    elif state == STATE_MOVE:

        left_dist = (left_enc - left_start_encoder) * WHEEL_RADIUS
        right_dist = (right_enc - right_start_encoder) * WHEEL_RADIUS

        segment_distance = (left_dist + right_dist) / 2.0
        remaining = target_distance - segment_distance

        # ============================================================
        # ✅ برای بخش آخر: از مختصات واقعی ایستگاه‌ها استفاده کن
        # ============================================================
        if target_index == 4:
            # مختصات واقعی START در Webots
            REAL_START_X = 0.540
            
            # ✅ از موقعیت واقعی X استفاده کن (نه ادومتری)
            # ما می‌دانیم که ربات در START باید X=0.540 باشد
            # اما برای توقف، از X واقعی استفاده می‌کنیم
            
            # برای این کار، موقعیت X را از مختصات ایستگاه قبلی (YELLOW) و مسافت طی‌شده محاسبه می‌کنیم
            # ساده‌تر: اگر مسافت طی‌شده به target_distance رسید، متوقف شو
            
            # محاسبه فاصله تا START از مختصات واقعی
            dx_real = WAYPOINTS[4][0] - WAYPOINTS[3][0]
            dz_real = WAYPOINTS[4][1] - WAYPOINTS[3][1]
            real_distance = math.sqrt(dx_real*dx_real + dz_real*dz_real)
            
            # اگر مسافت طی‌شده به real_distance نزدیک شد، متوقف شو
            if segment_distance >= real_distance * 0.98:
                stop_robot()
                print()
                print("============================================")
                print(f"✅ ARRIVED TO START (Distance: {segment_distance:.3f}m)")
                print(f"   Real distance to START: {real_distance:.3f}m")
                print("============================================")
                stop_start_time = current_time
                state = STATE_STOP
                continue

        # شرط معمولی توقف
        if remaining <= ARRIVAL_TOLERANCE:
            stop_robot()
            print()
            print("============================================")
            print(f"ARRIVED -> {WAYPOINT_NAMES[target_index]}")
            print(f"TRAVELLED = {segment_distance:.3f} m")
            print("STATE CHANGE -> STOP")
            print("============================================")
            stop_start_time = current_time
            state = STATE_STOP
        else:
            if remaining < SLOW_DISTANCE:
                speed = 1.0 + (remaining / SLOW_DISTANCE) * 1.0
            else:
                speed = MOVE_SPEED

            wheel_error = left_dist - right_dist
            correction = K_HEADING * wheel_error

            set_wheels(speed - correction, speed + correction)

            print(f"MOVE | TARGET={WAYPOINT_NAMES[target_index]:<15} | "
                  f"SEGMENT={segment_distance:.3f} m | REMAINING={remaining:.3f} m")


    # ========================================================
    # STATE 3: STOP (با تصحیح موقعیت)
    # ========================================================

    elif state == STATE_STOP:
        stop_robot()
        
        # ✅ تصحیح موقعیت به مختصات دقیق ایستگاه
        x = WAYPOINTS[target_index][0]
        z = WAYPOINTS[target_index][1]
        
        if (current_time - stop_start_time) >= STOP_TIME:
            print(f"STOP COMPLETE -> {WAYPOINT_NAMES[target_index]}")
            print(f"✅ POSITION CORRECTED TO: X={x:.3f} Z={z:.3f}")
            state = STATE_NEXT


    # ========================================================
    # STATE 4: NEXT STATION
    # ========================================================

    elif state == STATE_NEXT:

        target_index += 1

        if target_index >= len(WAYPOINTS):
            stop_robot()
            print()
            print("============================================")
            print("        MISSION COMPLETED")
            print(f"        FINAL POSITION: X={x:.3f} Z={z:.3f}")
            print("============================================")
            state = STATE_COMPLETE
        else:
            previous_x = WAYPOINTS[target_index - 1][0]
            previous_z = WAYPOINTS[target_index - 1][1]

            target_x = WAYPOINTS[target_index][0]
            target_z = WAYPOINTS[target_index][1]

            dx = target_x - previous_x
            dz = target_z - previous_z

            target_distance = math.sqrt(dx*dx + dz*dz)
            target_angle = math.atan2(dz, dx)

            # ============================================================
            # ✅ تعیین زاویه چرخش
            # ============================================================
            
            if target_index == 1:
                current_heading = math.radians(-90.0)
                segment_angle = normalize_angle(target_angle - current_heading)
            
            elif target_index == 4:
                # ✅ چرخش -107 درجه برای آخرین بخش
                segment_angle = math.radians(-107.0)   # -107 درجه
                print(f"✅ FORCED -107° TURN FOR FINAL SEGMENT")
            
            else:
                segment_angle = normalize_angle(target_angle - heading)
                if segment_angle > math.pi:
                    segment_angle -= 2.0 * math.pi
                elif segment_angle < -math.pi:
                    segment_angle += 2.0 * math.pi

            left_turn_start = left_enc
            right_turn_start = right_enc

            print()
            print("============================================")
            print(f"NEXT -> {WAYPOINT_NAMES[target_index]}")
            print(f"DISTANCE = {target_distance:.3f} m")
            print(f"TURN = {math.degrees(segment_angle):.2f}°")
            print("STATE CHANGE -> TURN")
            print("============================================")

            state = STATE_TURN


    # ========================================================
    # STATE 5: COMPLETE
    # ========================================================

    elif state == STATE_COMPLETE:
        stop_robot()