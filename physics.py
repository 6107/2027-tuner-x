import wpilib
import wpilib.simulation
import wpimath.geometry as geometry

from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.system.plant import LinearSystemId
from pyfrc.physics.core import PhysicsInterface
#
# This base file was created by using Chrome to request what the 2027 alpha-7 simulation
# file (physics.py) would look like with a Swerve Drive made up of SDS MK5i modules with
# a Kraken X60 drive motor and Kraken X44 steer motor and a CanC0der.
#
#
# Approximate Gear Ratios for SDS MK4i (Adjust based on your specific L1, L2, or L3 choice)
# For example, MK4i L2 uses 6.75:1 for drive and 150/7:1 (21.43:1) for steering
DRIVE_GEAR_RATIO = 6.75
STEER_GEAR_RATIO = 150.0 / 7.0

# Wheel radius in meters (approx 2 inches)
WHEEL_RADIUS = 0.0508


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller
        self.robot = robot

        # 1. Access the swerve subsystem from your robot instance
        # Assumes your main robot class has an attribute named 'drivetrain' or similar
        self.swerve_subsystem = robot.drivetrain

        # 2. Re-create or fetch kinematics references from the robot
        # This mirrors the layout of your 4 modules
        # Coordinates in meters: front_left, front_right, back_left, back_right
        self.kinematics: SwerveDrive4Kinematics = self.swerve_subsystem.kinematics

        # 3. Create physics plants for the motor profiles
        # Kraken X60 (Drive) and Kraken X44 (Steer) J-kg-m² constants
        # Using standard FlywheelSim models for velocity/position estimation
        self.drive_plant = LinearSystemId.flywheel_system(
            wpilib.simulation.DCMotor.kraken_x60(1), J=0.02, G=DRIVE_GEAR_RATIO
        )
        self.steer_plant = LinearSystemId.flywheel_system(
            wpilib.simulation.DCMotor.kraken_x44(1), J=0.004, G=STEER_GEAR_RATIO
        )

        # Track simulated positions internally (4 modules)
        self.module_positions = [0.0, 0.0, 0.0, 0.0]
        self.module_angles = [0.0, 0.0, 0.0, 0.0]

        # Initialize gyro simulation handle
        self.gyro_sim = wpilib.simulation.AnalogGyroSim(0)  # or NavX/Pigeon simulation equivalent

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Updates the simulation loop. 2027 uses snake_case and float for tm_diff.
        """
        # Array to collect state updates
        module_states = []

        # Loop through your 4 physical modules to read outputs and simulate physics
        for i, module in enumerate(self.swerve_subsystem.modules):
            # 1. Fetch applied voltages from your drive & steer motors
            # 2027 Alpha strictly maps these to snake_case methods
            drive_voltage = module.get_drive_motor_voltage()
            steer_voltage = module.get_steer_motor_voltage()

            # 2. Advance the individual motor physical simulation plants
            # (If you aren't using CTRE phoenix6's built-in sim mechanism directly)
            # Calculate wheel velocity (rad/s) and steer velocity (rad/s)
            # In a full deployment, you'd feed these back into SimState objects:
            # module.drive_motor.sim_state.set_supply_voltage(drive_voltage)

            # Simple math model backup:
            drive_velocity_rad_per_sec = (drive_voltage / 12.0) * wpilib.simulation.DCMotor.kraken_x60(
                1).free_speed_rad_per_sec / DRIVE_GEAR_RATIO
            steer_velocity_rad_per_sec = (steer_voltage / 12.0) * wpilib.simulation.DCMotor.kraken_x44(
                1).free_speed_rad_per_sec / STEER_GEAR_RATIO

            # Integrate velocities into positions over the current time step
            self.module_positions[i] += (drive_velocity_rad_per_sec * WHEEL_RADIUS) * tm_diff
            self.module_angles[i] += steer_velocity_rad_per_sec * tm_diff

            # Pack it into WPIMath structures for kinematics
            state = geometry.SwerveModuleState(
                speed=drive_velocity_rad_per_sec * WHEEL_RADIUS,
                angle=geometry.Rotation2d(self.module_angles[i])
            )
            module_states.append(state)

            # 3. Push data back into your simulated CANcoders and Encoders
            # module.cancoder.sim_state.set_raw_position(self.module_angles[i])
            # module.drive_encoder.sim_state.set_raw_position(self.module_positions[i])

        # 4. Use kinematics to extract forward, strafe, and angular robot chassis velocities
        chassis_speeds = self.kinematics.to_chassis_speeds(
            module_states[0], module_states[1], module_states[2], module_states[3]
        )

        # 5. Update Gyro/Odometry rotation based on calculated angular velocity
        # chassis_speeds.omega is in radians per second
        degrees_turned = (chassis_speeds.omega * (180.0 / 3.14159)) * tm_diff

        # Pull your current simulated gyro angle, add the delta, and push back
        current_angle = self.gyro_sim.get_angle()
        self.gyro_sim.set_angle(current_angle + degrees_turned)

        # 6. Finally, update pyfrc's field visualization tool representation
        # This will update the graphic position of the robot icon on the desktop simulation field
        self.physics_controller.drive_chassis_speeds(chassis_speeds, tm_diff)
