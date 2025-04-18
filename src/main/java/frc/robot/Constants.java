package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants
{
    public static class AdvantageKit
    {
        public static final Mode SIM_MODE     = Mode.SIM;
        public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

        public static enum Mode
        {
            /** Running on a real robot. */
            REAL,

            /** Running a physics simulator. */
            SIM,

            /** Replaying from a log file. */
            REPLAY
        }
    }

    public static class AIO
    {
        public static final int FL_ENCODER = 11;
        public static final int FR_ENCODER = 13;
        public static final int BL_ENCODER = 12;
        public static final int BR_ENCODER = 14;
    }

    public static class Autos
    {
        public static final PathPlannerAuto LEFT_1_CORAL_AUTO   = new PathPlannerAuto("Left_J");
        public static final PathPlannerAuto MIDDLE_1_CORAL_AUTO = new PathPlannerAuto("Middle_G");
        public static final PathPlannerAuto RIGHT_1_CORAL_AUTO  = new PathPlannerAuto("Right_E");
        public static final PathPlannerAuto RIGHT_2_CORAL_AUTO  = new PathPlannerAuto("Right_DC");
    }

    public static class CAN
    {
        public static final int FL_DRIVE          = 10;
        public static final int FR_DRIVE          = 9;
        public static final int BL_DRIVE          = 5;
        public static final int BR_DRIVE          = 7;
        public static final int FL_TURN           = 4;
        public static final int FR_TURN           = 3;
        public static final int BL_TURN           = 2;
        public static final int BR_TURN           = 6;
        public static final int LEAD_ELEVATOR     = 15;
        public static final int FOLLOWER_ELEVATOR = 16;
        public static final int MANIPULATOR       = 17;
    }

    public static class Controls
    {
        public static final double JOYSTICK_DEADBAND = 0.1;
    }

    public static class Dashboard
    {
        public static final double LOW_BATTERY_VOLTAGE        = 11.5;
        public static final double LOW_BATTERY_TIME_THRESHOLD = 5.0; // seconds
    }

    public static class DIO
    {
        public static final int MANIPULATOR_LIGHT_SENSOR_END   = 9;
        public static final int MANIPULATOR_LIGHT_SENSOR_START = 8;
    }

    public static class Drive
    {
        public static final double          TRACK_WIDTH                   = Units.inchesToMeters(24);
        public static final double          WHEEL_BASE                    = Units.inchesToMeters(24);
        public static final double          DRIVE_BASE_RADIUS             = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
        public static final Translation2d[] MODULE_TRANSLATIONS           = new Translation2d[] { new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0), new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
                new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0), new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0) };
        public static final Rotation2d      FL_ZERO_ROTATION              = Rotation2d.fromRotations(0.074707);
        public static final Rotation2d      FR_ZERO_ROTATION              = Rotation2d.fromRotations(0.129883); // Need to adapt for our hardware
        public static final Rotation2d      BL_ZERO_ROTATION              = Rotation2d.fromRotations(0.084229);
        public static final Rotation2d      BR_ZERO_ROTATION              = Rotation2d.fromRotations(-0.036865);
        public static final int             DRIVE_MOTOR_CURRENT_LIMIT     = 60;
        public static final double          WHEEL_RADIUS                  = Units.inchesToMeters(2);
        public static final double          DRIVE_MOTOR_REDUCTION         = 5.67;
        public static final boolean         DRIVE_INVERTED                = false;
        public static final DCMotor         DRIVE_GEARBOX                 = DCMotor.getNEO(1);
        public static final double          DRIVE_KP                      = 0.08;
        public static final double          DRIVE_KD                      = 0.0;
        public static final double          DRIVE_KS                      = 0.0;
        public static final double          DRIVE_KV                      = 0.1;
        public static final double          DRIVE_SIM_KP                  = 0.2;
        public static final double          DRIVE_SIM_KD                  = 0.0;
        public static final double          DRIVE_SIM_KS                  = 0.0;
        public static final double          DRIVE_SIM_KV                  = 0.0789;
        public static final boolean         TURN_INVERTED                 = true;
        public static final int             TURN_MOTOR_CURRENT_LIMIT      = 60;
        public static final double          TURN_MOTOR_REDUCTION          = 12.1;
        public static final DCMotor         TURN_GEARBOX                  = DCMotor.getNEO(1);
        public static final double          TURN_KP                       = 1.75;
        public static final double          TURN_KD                       = 0.0;
        public static final double          TURN_SIM_KP                   = 8.0;
        public static final double          TURN_SIM_KD                   = 0.0;
        public static final double          ODOMETRY_FREQUENCY            = 100.0; // ms
        public static final double          ROTATE_KP                     = 0.005;
        public static final double          ROTATE_KD                     = 0.0;
        public static final double          MAX_SPEED_ELEVATOR_MULTIPLIER = 1;
        public static final double          MIN_SPEED_ELEVATOR_MULTIPLIER = 0.3;
        public static final double          MAX_SPEED_ELEVATOR_HEIGHT     = Constants.Elevator.L1_HEIGHT;
        public static final double          MIN_SPEED_ELEVATOR_HEIGHT     = Constants.Elevator.L3_HEIGHT;
        public static final double          SPEED_ELEVATOR_M              = (MAX_SPEED_ELEVATOR_MULTIPLIER - MIN_SPEED_ELEVATOR_MULTIPLIER) / (MAX_SPEED_ELEVATOR_HEIGHT - MIN_SPEED_ELEVATOR_HEIGHT);
        public static final double          SPEED_ELEVATOR_B              = MAX_SPEED_ELEVATOR_MULTIPLIER - SPEED_ELEVATOR_M * MAX_SPEED_ELEVATOR_HEIGHT;
        public static final double          MAX_LINEAR_SPEED              = 2.4; // m/s
        public static final double          MAX_ANGULAR_SPEED             = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
        public static final double          SPEED_MOTION_THRESHOLD        = 0.02 * MAX_LINEAR_SPEED;
        public static final double          ROTATION_MOTION_THRESHOLD     = 0.02 * MAX_ANGULAR_SPEED;
        public static final double          MAX_SNAP_SPEED_PERCENTAGE     = 0.7;
    }

    public static class Field
    {
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT   = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        public static final Rotation2d          BLUE_REEF_ANGLE_ONE      = getTagAngle(18);
        public static final Rotation2d          BLUE_REEF_ANGLE_TWO      = getTagAngle(17);
        public static final Rotation2d          BLUE_REEF_ANGLE_THREE    = getTagAngle(22);
        public static final Rotation2d          BLUE_REEF_ANGLE_FOUR     = getTagAngle(21);
        public static final Rotation2d          BLUE_REEF_ANGLE_FIVE     = getTagAngle(20);
        public static final Rotation2d          BLUE_REEF_ANGLE_SIX      = getTagAngle(19);
        public static final Rotation2d          RED_REEF_ANGLE_ONE       = getTagAngle(7);
        public static final Rotation2d          RED_REEF_ANGLE_TWO       = getTagAngle(8);
        public static final Rotation2d          RED_REEF_ANGLE_THREE     = getTagAngle(9);
        public static final Rotation2d          RED_REEF_ANGLE_FOUR      = getTagAngle(10);
        public static final Rotation2d          RED_REEF_ANGLE_FIVE      = getTagAngle(11);
        public static final Rotation2d          RED_REEF_ANGLE_SIX       = getTagAngle(6);
        public static final Rotation2d          BLUE_LEFT_STATION_ANGLE  = getTagAngle(13).plus(Rotation2d.fromDegrees(180));
        public static final Rotation2d          BLUE_RIGHT_STATION_ANGLE = getTagAngle(12).plus(Rotation2d.fromDegrees(180));
        public static final Rotation2d          RED_LEFT_STATION_ANGLE   = getTagAngle(1).plus(Rotation2d.fromDegrees(180));
        public static final Rotation2d          RED_RIGHT_STATION_ANGLE  = getTagAngle(2).plus(Rotation2d.fromDegrees(180));
        public static final Rotation2d          BLUE_PROCESSOR_ANGLE     = getTagAngle(16);
        public static final Rotation2d          RED_PROCESSOR_ANGLE      = getTagAngle(3);

        private static Rotation2d getTagAngle(int tagID)
        {
            return APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get().getRotation().toRotation2d().rotateBy(new Rotation2d(Math.PI));
        }
    }

    public static class Elevator
    {
        public static final double  RAW_SENSOR_MIN               = 0.634;
        public static final double  RAW_SENSOR_MAX               = 0.394;
        public static final double  MIN_EXTENSION                = 1.0;
        public static final double  MAX_EXTENSION                = 27.0;
        public static final double  EXTENSION_KP                 = 3;
        public static final double  EXTENSION_KI                 = 0.0;
        public static final double  EXTENSION_KD                 = 0.025;
        public static final double  EXTENSION_TOLERANCE          = 1.0;
        public static final double  STOW_HEIGHT                  = MIN_EXTENSION;
        public static final double  INTAKE_HEIGHT                = MIN_EXTENSION + 7;
        public static final double  L1_HEIGHT                    = MIN_EXTENSION + 7;
        public static final double  L2_HEIGHT                    = MIN_EXTENSION + 10;
        public static final double  L3_HEIGHT                    = MIN_EXTENSION + 15.8;
        public static final double  L4_HEIGHT                    = MIN_EXTENSION + 25;
        public static final double  MAX_UPWARDS_SPEED            = 1;
        public static final double  MAX_DOWNWARDS_SPEED          = 0.5;
        public static final double  EXTENSION_SCALE              = (MAX_EXTENSION - MIN_EXTENSION) / (RAW_SENSOR_MAX - RAW_SENSOR_MIN);
        public static final double  EXTENSION_MOTOR_REDUCTION    = 6.0;
        public static final DCMotor ELEVATOR_GEARBOX             = DCMotor.getNEO(2);
        public static final double  EXTENSION_OFFSET             = MIN_EXTENSION - EXTENSION_SCALE * RAW_SENSOR_MIN;// Does nothing
        public static final double  ELEVATOR_MASS                = 15.875;
        public static final double  ELEVATOR_DRUM_RADIUS         = 0.0223139;
        public static final double  ELEVATOR_FEED_FORWARD        = 0.7;
        public static final double  ELEVATOR_MODIFICATION_HEIGHT = 0.5;
        public static final double  WAIT_TIME                    = 1;
        public static final double  MAX_ACCELERATION             = 30;
        public static final double  MAX_VELOCITY                 = 70;
    }

    public static class General
    {
        public static final double LOOP_PERIOD_SECS = 0.02;
        public static final double MOTOR_VOLTAGE    = 12.0;
    }

    public static class PathPlanner
    {
        public static final double       DRIVE_KP      = 0.003;
        public static final double       DRIVE_KD      = 0.0;
        public static final double       TURN_KP       = 0.005;
        public static final double       TURN_KD       = 0.0;
        public static final double       WHEEL_COF     = 1.2;
        public static final double       ROBOT_MASS    = 60;
        public static final double       ROBOT_MOI     = 6.883;
        public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(Drive.WHEEL_RADIUS, Drive.MAX_LINEAR_SPEED, WHEEL_COF, Drive.DRIVE_GEARBOX, Drive.DRIVE_MOTOR_REDUCTION, Drive.DRIVE_MOTOR_CURRENT_LIMIT, 1);
        public static final RobotConfig  ROBOT_CONFIG  = new RobotConfig(ROBOT_MASS, ROBOT_MOI, MODULE_CONFIG, Drive.MODULE_TRANSLATIONS);
    }
}
