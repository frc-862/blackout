package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.thunder.math.InterpolationMap;
import frc.thunder.pathplanner.com.pathplanner.lib.auto.PIDConstants;

/**
 * Class to hold all of the constants for the robot
 */
public final class Constants {

    // Spark max voltage compensation
    public static final double VOLTAGE_COMPENSATION = 12d;

    // Path to the blackout directory
    public static final Path BLACKOUT_PATH = Paths.get("home/lvuser/blackout");

    // Check if we're on blackout
    public static final boolean isBlackout() { // MAKE DEFAULT?
        return BLACKOUT_PATH.toFile().exists();
    }

    // Check if we're on gridlock
    public static final boolean isGridlock() {
        return !isBlackout();
    }

    public static final double COMP_LOG_PERIOD = .33;

    // Enum of possible game pieces
    public enum GamePiece {
        CONE, CUBE, NONE
    }

    // Constants for xbox controlers
    public static final class ControllerConstants {
        // Ports for the controllers
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int COPILOT_CONTROLLER_PORT = 1;
        public static final int BUTTON_PAD_CONTROLLER_PORT = 2;

        // Deadband, min, and max power for the controllers
        public static final double DEADBAND = 0.1d;
        public static final double MIN_POWER = 0d;
        public static final double MAX_POWER = 1d;
    }

    // Constants for our system tests
    public static final class SystemTestConstants {
        // Drive Test Variables
        public static final int DEGREES_INTERVAL_INCREASE = 30;
        public static final int ANGLE_DEAD_ZONE = 3;
        public static final int MAX_ROTATIONS_PER_DIRECTION = 2;
    }

    // Constants for our drivetrain
    public static final class DrivetrainConstants {
        // Our drivetrain track width and Wheelbase
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20.8125d);
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(20.8125d);

        // Module resting/default angles
        public static final double FRONT_LEFT_RESTING_ANGLE = Math.toRadians(-45d);
        public static final double FRONT_RIGHT_RESTING_ANGLE = Math.toRadians(45d);
        public static final double BACK_LEFT_RESTING_ANGLE = Math.toRadians(45d);
        public static final double BACK_RIGHT_RESTING_ANGLE = Math.toRadians(-45d);

        // Our max voltage, velocity, angular velocity, and angular acceleration
        public static final double MAX_VOLTAGE = 12;
        // public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676.0 / 60.0 *
        // SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
                / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        DRIVETRAIN_WHEELBASE_METERS / 2.0);
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = +MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                * 2 * Math.PI / 5;

        // Module configuration constants
        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int STEER_CURRENT_LIMIT = 30;
        public static final double NOMINAL_VOLTAGE = 12d;

        public static final double LOG_PERIOD = 0.18;

        public static final double SLOW_MODE_TRANSLATIONAL_MULT = 0.7;
        public static final double SLOW_MODE_ROTATIONAL_MULT = 0.5;

        // Pigeon heading offset
        public static final Rotation2d HEADING_OFFSET = Rotation2d.fromDegrees(90);

        // Standard dev for robot pose
        public static final Matrix<N3, N1> STANDARD_DEV_POSE_MATRIX = VecBuilder.fill(0.1, 0.1, 0.1);

        // Gains vaules for PIDControllers
        public static final class Gains {
            public static final double kP = 0.15;// .116d;
            public static final double kI = 0d;
            public static final double kD = 0d;

            public static final double kF = 0.225;// 229d;
        }

        // Gains vaules for theta PIDControllers
        public static final class ThetaGains {
            public static final double kP = 0d;
            public static final double kI = 0d;
            public static final double kD = 0d;
        }

        // PID gains for our heading compensation
        public static final class HeadingGains {
            public static final double kP = 0.005d;
            public static final double kI = 0d;
            public static final double kD = 0d;
        }

        // Steer offsets for our modules
        public static final class Offsets {
            // Gridlocks swerve module absolute encoder offsets
            public static final class Gridlock {
                public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(0);
                public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(0);
                public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(0);
                public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(0);
            }

            // Blackouts swerve module absolute encoder offsets
            public static final class Blackout { // TODO Need offsets
                public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(257.871);
                public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(222.012);
                public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(129.287);
                public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(63.809);
            }
        }
    }

    public static final class LimelightConstants {
        public static final String FRONT_NAME = "limelight-front";
        public static final Pose3d FRONT_POSE = new Pose3d(.1, 0.28, 0.72, new Rotation3d(0, 0, 0)); // Position on
                                                                                                     // robot
        public static final double CUBE_OFFSET = 0.0; // TODO find this value
    }

    public static final class AutoAlignConstants {
        public static final double RkP = 0.01d;
        public static final double RkI = 0.00d;
        public static final double RkD = 0.01d;

        public static final double YkP = -0.05d;
        public static final double YkI = 0.00d;
        public static final double YkD = -0.03d;

        public static final double TARGET_ANGLE = 180d;

        public static final double TOLERANCE = 1d;

    }

    public static final class CollectorConstants {
        public static final boolean MOTOR_INVERT = true;

        public static final int CURRENT_LIMIT = 0; // :P

        public static final double HOLD_POWER = 0.05;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;

        public static final double STALL_POWER = 35d; // Used to detect wether or not the collector
                                                      // is stalling meaning it has a game piece
        // TODO Test Amount ^

        public static final double LOG_PERIOD = 0.22;

        // public static final double MOTOR_kP = 0.00d; // TODOnt NEED THESE
        // public static final double MOTOR_kI = 0.00d;
        // public static final double MOTOR_kD = 0.00d;

        // public static final double MOTOR_kS = 0.00d;
        // public static final double MOTOR_kA = 0.00d;
        // public static final double MOTOR_kV = 0.00d;

        // public static final double MAX_RPM = 5820;
    }

    public static final class WristConstants {

        // Motor configuration constants
        public static final boolean MOTOR_INVERT = false;
        public static final boolean ENCODER_INVERT = false;

        public static final int CURRENT_LIMIT = 40;
        public static final int STALL_CURRENT = 30; // TODO GET AT zeroing speed
        public static final double IS_MOVING_THRESHHOLD = 0.05d; // TODO TEST

        public static final IdleMode IDLE_MODE = IdleMode.kCoast;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;

        public static final double WRIST_TOLERANCE = 4d;

        // PID gains for our wrist going up and down
        public static final double UP_kP = 0.15d;
        public static final double UP_kI = 0.0d;
        public static final double UP_kD = 0.000d;

        public static final double DOWN_kP = 0.07d;
        public static final double DOWN_kI = 0d;
        public static final double DOWN_kD = 0d;

        // Min/max angles in degrees
        public static final double MAX_ANGLE = 118d;
        public static final double MIN_ANGLE = 0;

        // Min and Max power
        public static final double MIN_POWER = -1d; // TODO implement and check
        public static final double MAX_POWER = 1d;

        public static final double LOG_PERIOD = 0.24;

        // Offsets in degrees
        public static final double OFFSET = 0d; // TODO GET NEW

        // Zero speed
        public static final double ZERO_SPEED = 0.1d; // TODO test

        // Conversion factor for our wrist, multiply this by the navite units to get
        // degrees
        public static final double POSITION_CONVERSION_FACTOR = 360 / 160; // TODO check

        // Interpolation map for our arm Feedforward values to make sure we have enough
        // minimum
        // power to move the arm
        public static InterpolationMap WRIST_KF_MAP = new InterpolationMap() {
            {
                // put(-90d, 0d);
                // put(-45d, -0.008d);
                // put(0d, 0.017d);
                // put(45d, 0.01d);
                // put(90d, 0d);
                // put(135d, 0.008d);
                put(0d, 0d);
            }
        };
    }

    // RobotMap Constants
    public static final class RobotMap {
        // CAN IDs
        public static final class CAN {
            // Pigeon IMU ID
            public static final int PIGEON_ID = 23;
            // Power distrobution hub ID
            public static final int PDH = 21;

            // Front left CanIDs
            public static final int FRONT_LEFT_DRIVE_MOTOR = 1;
            public static final int FRONT_LEFT_AZIMUTH_MOTOR = 2;
            public static final int FRONT_LEFT_CANCODER = 31;
            // Front right CanIDs
            public static final int FRONT_RIGHT_DRIVE_MOTOR = 3;
            public static final int FRONT_RIGHT_AZIMUTH_MOTOR = 4;
            public static final int FRONT_RIGHT_CANCODER = 32;
            // Back right CanIDs
            public static final int BACK_RIGHT_DRIVE_MOTOR = 5;
            public static final int BACK_RIGHT_AZIMUTH_MOTOR = 6;
            public static final int BACK_RIGHT_CANCODER = 33;
            // Back left CanIDs
            public static final int BACK_LEFT_DRIVE_MOTOR = 7;
            public static final int BACK_LEFT_AZIMUTH_MOTOR = 8;
            public static final int BACK_LEFT_CANCODER = 34;

            // COLLECTOR
            public static final int COLLECTOR_MOTOR = 10;
            // WRIST
            public static final int WRIST_MOTOR = 9;

            // CANdle
            public static final int CANDLE = 22;
        }

        public static final class PWM {
            public static final int SERVO = 0;
        }
    }

    // Constants used for auto balancing
    public static final class AutoBalanceConstants {
        // Magnitude for being balance
        public static final double BALANCED_MAGNITUDE = 2.5;

        // Upper and lower magnitude thresholds for checking if we are on the charge
        // station at all
        public static final double UPPER_MAGNITUDE_THRESHOLD = 11;
        public static final double LOWER_MAGNITUDE_THRESHOLD = 7;
        // Min and max speeds for our auto balance
        public static final double MIN_SPEED_THRESHOLD = 0.35;
        public static final double MAX_SPEED_THRESHOLD = 1.5;

        // Delay time for our auto balance after falling
        public static final double DELAY_TIME = 1.5;

        // Target X position for the middle of the charge station
        public static final double TARGET_X = 3.85;// 3.93;

        // log period for autobalance
        public static final double LOG_PERIOD = 0.2;

        // Gains for our auto balance
        public static final double kP = 2;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    // Constants for the LEDs
    public static final class LedConstants {
        public static final double BRIGHTNESS = .25;
        // public static final LEDStripType STRIP_TYPE = LEDStripType.RGB;
        public static final int LED_LENGTH = 150;

        public static enum LEDStates {
            wantsCone, wantsCube, hasCone, hasCube, override, noPiece
        }
    }

    // Constants for vision
    public static final class VisionConstants {
        // Represents camera FOV from center to edge
        public static final double HORIZ_CAMERA_FOV = 29.8d;

        // Arbitrary value for how close the robot needs to be to the target (in
        // degrees)
        public static final double HORIZ_DEGREE_TOLERANCE = 3d;

        // Standard deviation for vision, heading is 1000 becuase were using pigeon, so
        // i dont want
        // to use vision heading
        public static final Matrix<N3, N1> STANDARD_DEV_VISION_MATRIX = VecBuilder.fill(0.6, 0.6, 0.6);

        // Distance from the center of the field, used for getIsolatedTagPose()
        public static final double ISOLATEDTAGXOFFSET = 7.24;

        public static final double ISOLATEDTAGYOFFSET = 1.07;

        public static final InterpolationMap visionStandardDevMap = new InterpolationMap() {
            {
                put(0d, 0.3);
                put(2d, 0.4);
                put(5d, 0.5);
                put(7d, 1d);
                put(8d, 1.5d);
                put(12d, 10d);
            }
        };

    }

    // Constants for autonomous
    public static final class AutonomousConstants {
        // Path planner PIDConstants
        public static final PIDConstants DRIVE_PID_CONSTANTS = new PIDConstants(2.5, 0, 0); // Drive velocity PID 10.5
        public static final PIDConstants THETA_PID_CONSTANTS = new PIDConstants(4, 0, 0); // Rotation PID 7
        public static final PIDConstants POSE_PID_CONSTANTS = new PIDConstants(0, 0, 0); // X and Y position PID

        // Max velocity and acceleration for the path planner
        public static final double MAX_VELOCITY = 2;
        public static final double MAX_ACCELERATION = 1;
        public static final double SERVO_UP = -1d;
        public static final double SERVO_DOWN = 0.2d;

    }

    // Constants for autoAlign
    public static final class AutoScoreConstants {

        public static final class BluePoints {
            public static final Pose2d SLOT_1_POSE = new Pose2d(2.44, 4.9, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_2_POSE = new Pose2d(2.44, 4.35, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_3_POSE = new Pose2d(2.44, 3.8, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_4_POSE = new Pose2d(2.44, 3.25, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_5_POSE = new Pose2d(2.44, 2.7, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_6_POSE = new Pose2d(2.44, 2.15, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_7_POSE = new Pose2d(2.44, 1.6, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_8_POSE = new Pose2d(2.44, 1.05, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_9_POSE = new Pose2d(2.44, 0.5, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_10_POSE = new Pose2d(14.23, 2.65, Rotation2d.fromDegrees(90));
        }

        public static final class RedPoints {
            public static final Pose2d SLOT_10_POSE = new Pose2d(14.23, .09, Rotation2d.fromDegrees(-90));
            public static final Pose2d SLOT_9_POSE = new Pose2d(2.44, 7.3, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_8_POSE = new Pose2d(2.44, 6.75, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_7_POSE = new Pose2d(2.44, 6.2, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_6_POSE = new Pose2d(2.44, 5.65, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_5_POSE = new Pose2d(2.44, 5.1, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_4_POSE = new Pose2d(2.44, 4.55, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_3_POSE = new Pose2d(2.44, 4.0, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_2_POSE = new Pose2d(2.44, 3.45, Rotation2d.fromDegrees(180));
            public static final Pose2d SLOT_1_POSE = new Pose2d(2.44, 2.9, Rotation2d.fromDegrees(180));
        }

        public static enum SlotPosition { // Position of each colum of scoring nodes for AutoScoring
            slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8, slot9
        }

        public static final double MAX_ACCELERATION_MUL = 1;

        public static final double TOLERANCE_CUBE_ALLIGN_Y = 5d;
        public static final double TOLERANCE_CUBE_ALLIGN_Z = 5d;

        public static final double CONTROL_LENGTHS = 0.001;

        // Tolerance for auto align
        public static final double X_TOLERANCE = 7d;
        public static final double R_TOLERANCE = 5d;

        public static final double HORIZONTAL_MULTIPLIER = 1d;

        // Log period auto align
        public static final double LOG_PERIOD = 0.25;
    }

    public static final class WristAngles {

        public static final HashMap<wristStates, Double> angleMap() { // TODO TEST
            HashMap<wristStates, Double> angleMap = new HashMap<>();
            angleMap.put(wristStates.Ground, 3d);
            angleMap.put(wristStates.Stow, 118d);
            angleMap.put(wristStates.MidCube, 70d);
            angleMap.put(wristStates.HighCube, 115d);
            return angleMap;
        } 

        public static final HashMap<wristStates, Double> shootMap() { // TODO NOT USED
            HashMap<wristStates, Double> shootMap = new HashMap<>();
            shootMap.put(wristStates.Stow, CollectorConstants.HOLD_POWER);
            shootMap.put(wristStates.Ground, -0.25);
            shootMap.put(wristStates.MidCube, -0.75);
            shootMap.put(wristStates.HighCube, -1.0);
            return shootMap;
        }

        public static enum wristStates {
            Ground, MidCube, HighCube, Stow
        }

    }
}
