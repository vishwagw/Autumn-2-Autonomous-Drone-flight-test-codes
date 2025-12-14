"""
takeoff1.py

Webots controller to test a simple takeoff/hover/land sequence adapted from
an onboard DroneKit script. This controller uses a simple PID on altitude
(using GPS Y value) to adjust propeller motor velocities. It contains
reasonable defaults and command-line options to override target altitude,
hover time, and motor name discovery.

Assumptions / notes:
- The robot uses four motors/rotors whose names are detected from common
  naming patterns. If your model uses different names, pass them with
  --motors "m1,m2,m3,m4".
- A 'gps' device is used to read altitude (GPS Y value). If your model
  uses a different device, modify the code accordingly.
- Motor velocity units and hover base velocity are model-specific. Tune
  --base-velocity and PID gains if needed.

Usage inside Webots (controller): place this file in the controller folder
and set the controller for the robot. You can also pass arguments via the
Webots supervisor/controller args.
"""

import sys
import time
import argparse

try:
    # Webots controller API
    from controller import Robot
    from controller import Motor
    from controller import GPS
except Exception:
    # If running outside Webots for static checks, define minimal stubs
    Robot = None
    Motor = None
    GPS = None


COMMON_MOTOR_SETS = [
    ['rotor_1', 'rotor_2', 'rotor_3', 'rotor_4'],
    ['rotor1', 'rotor2', 'rotor3', 'rotor4'],
    ['propeller1', 'propeller2', 'propeller3', 'propeller4'],
    ['motor_front_left', 'motor_front_right', 'motor_back_left', 'motor_back_right'],
    ['motor_front_left', 'motor_front_right', 'motor_rear_left', 'motor_rear_right'],
    ['motor1', 'motor2', 'motor3', 'motor4']
]


def discover_motors(robot):
    """Try to find 4 motors using common naming patterns."""
    for candidate in COMMON_MOTOR_SETS:
        motors = []
        try:
            for name in candidate:
                m = robot.getDevice(name)
                if m is None:
                    raise RuntimeError('missing')
                motors.append(m)
        except Exception:
            continue
        return motors
    # Fallback: try motor1..motor8 and pick first 4 that exist
    motors = []
    for i in range(1, 9):
        try:
            m = robot.getDevice(f'motor{i}')
        except Exception:
            m = None
        if m is not None:
            motors.append(m)
        if len(motors) == 4:
            return motors
    return None


def find_device(robot, names):
    for n in names:
        try:
            d = robot.getDevice(n)
        except Exception:
            d = None
        if d is not None:
            return d
    return None


def build_arg_parser():
    p = argparse.ArgumentParser(description='Webots takeoff test controller')
    p.add_argument('--target-alt', type=float, default=2.0, help='Target altitude in meters')
    p.add_argument('--hover', type=int, default=5, help='Seconds to hover')
    p.add_argument('--max-alt', type=float, default=5.0, help='Safety max altitude')
    p.add_argument('--base-velocity', type=float, default=40.0, help='Base motor velocity (tune per model)')
    p.add_argument('--kp', type=float, default=6.0, help='Altitude P gain')
    p.add_argument('--ki', type=float, default=0.0, help='Altitude I gain')
    p.add_argument('--kd', type=float, default=2.0, help='Altitude D gain')
    p.add_argument('--motors', type=str, default=None, help='Comma-separated motor names (in order)')
    p.add_argument('--gps-names', type=str, default='gps', help='Comma-separated gps device names to try')
    return p


def main():
    parser = build_arg_parser()
    args, unknown = parser.parse_known_args()

    if Robot is None:
        print('This controller must be run inside Webots. Exiting.')
        print('Parsed args:', args)
        return

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Discover motors
    motors = None
    if args.motors:
        names = [n.strip() for n in args.motors.split(',') if n.strip()]
        motors = []
        for name in names:
            try:
                m = robot.getDevice(name)
            except Exception:
                m = None
            if m is None:
                print(f'Could not find motor device: {name}')
                motors = None
                break
            motors.append(m)
        if motors is not None and len(motors) != 4:
            print('Expected 4 motors when passing --motors')
            motors = None

    if motors is None:
        motors = discover_motors(robot)

    if not motors:
        print('ERROR: Could not discover 4 motors. Use --motors to specify names.')
        return

    # Configure motors for velocity control
    for m in motors:
        try:
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
        except Exception:
            pass

    # Discover GPS for altitude
    gps_names = [n.strip() for n in args.gps_names.split(',') if n.strip()]
    gps = find_device(robot, gps_names)
    if gps:
        try:
            gps.enable(timestep)
        except Exception:
            pass
    else:
        print('WARNING: No GPS device found; altitude feedback unavailable')

    target_alt = args.target_alt
    hover_time = args.hover
    max_alt = args.max_alt

    # PID controller state
    kp = args.kp
    ki = args.ki
    kd = args.kd
    integral = 0.0
    last_error = 0.0
    base_velocity = args.base_velocity

    print('\n=== takeoff1.py starting ===')
    print('Target alt: %.2fm | Hover: %ds | Base vel: %.1f' % (target_alt, hover_time, base_velocity))

    # Helper to set motor velocities
    def set_all_vel(v):
        for mt in motors:
            try:
                mt.setVelocity(v)
            except Exception:
                pass

    # Safe takeoff ramp: slowly increase base velocity until we detect ascent
    print('\nRamping up motors...')
    vel = base_velocity * 0.2
    set_all_vel(vel)

    start_time = robot.getTime()
    last_time = start_time

    reached = False
    timeout = 60.0

    while robot.step(timestep) != -1:
        t = robot.getTime()
        dt = t - last_time if t - last_time > 0 else 1e-3

        # Read altitude from GPS (Y axis)
        alt = None
        if gps:
            try:
                vals = gps.getValues()
                # Webots uses Y as up
                alt = float(vals[1])
            except Exception:
                alt = None

        # PID control only when we have altitude
        if alt is not None:
            error = target_alt - alt
            integral += error * dt
            derivative = (error - last_error) / dt
            u = kp * error + ki * integral + kd * derivative
            vel = max(0.0, base_velocity + u)
            # clamp
            vel = min(500.0, vel)
            set_all_vel(vel)
            last_error = error

            print('t=%.1f alt=%.2f target=%.2f vel=%.2f' % (t, alt, target_alt, vel))

            # Safety: exceeded max altitude
            if alt > max_alt:
                print('SAFETY: exceeded max altitude %.2fm (now %.2fm)' % (max_alt, alt))
                break

            # Check reached
            if alt >= target_alt * 0.95:
                print('Reached target altitude: %.2fm' % alt)
                reached = True
                break
        else:
            # If no altitude available, just ramp
            vel = vel + 0.5
            set_all_vel(vel)
            print('t=%.1f vel=%.2f (no altitude feedback yet)' % (t, vel))

        if t - start_time > timeout:
            print('Takeoff timeout after %.0fs' % timeout)
            break

        last_time = t

    if not reached:
        print('Takeoff did not reach target. Attempting landing/stop sequence')
        # Smoothly reduce velocities
        for step in range(30):
            vel = max(0.0, vel * 0.9)
            set_all_vel(vel)
            if robot.step(timestep) == -1:
                break
        print('Motors slowed')
        return

    # Hover
    print('\nHovering for %d seconds...' % hover_time)
    hover_start = robot.getTime()
    while robot.step(timestep) != -1:
        t = robot.getTime()
        alt = None
        if gps:
            try:
                alt = float(gps.getValues()[1])
            except Exception:
                alt = None

        # simple altitude hold: small pid adjustments
        if alt is not None:
            error = target_alt - alt
            integral += error * (t - last_time)
            derivative = (error - last_error) / max(1e-3, (t - last_time))
            u = kp * error + ki * integral + kd * derivative
            vel = max(0.0, base_velocity + u)
            set_all_vel(vel)
            last_error = error
            last_time = t

        if t - hover_start >= hover_time:
            print('Hover complete')
            break

    # Land: reduce motor velocity gradually
    print('\nLanding: reducing motor speed')
    for i in range(60):
        vel = max(0.0, vel * 0.95)
        set_all_vel(vel)
        if robot.step(timestep) == -1:
            break

    print('Landed (motors set to zero). Finished takeoff1.py')


if __name__ == '__main__':
    main()
