package frc.robot.constants;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

import java.util.List;


public class Constants extends CommandBase {


    public static class Collector {
        public static final int MOTOR_ID = 7;

        public static final double  GEAR_RATIO = 2;
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * 2;
        public static final double WHEEL_REVS_PER_SEC_TO_VELOCITY = 8050;
        public static final int CUBE_BREAK_1_PORT = 2;
        public static final int CUBE_BREAK_2_PORT = 3;

        public static final double CONE_COLLECT_AMP_THRESHOLD = 12;
        public static final double CONE_AMPS_HOLDING_THRESHOLD = 10;

        public static final double INTAKE_SPEED = 10; //inches per second
        public static final double INTAKE_MOTOR_SPEED =
                INTAKE_SPEED / WHEEL_CIRCUMFERENCE * WHEEL_REVS_PER_SEC_TO_VELOCITY;
        public static final double INTAKE_MOTOR_SPEED_SLOW = INTAKE_MOTOR_SPEED/2;
        public static final double INTAKE_MOTOR_SPEED_VERY_SLOW = INTAKE_MOTOR_SPEED/4;


        public static final double EXPEL_SPEED = 10; // inches per second
        public static final double EXPEL_MOTOR_SPEED =
                EXPEL_SPEED / WHEEL_CIRCUMFERENCE * WHEEL_REVS_PER_SEC_TO_VELOCITY;
        public static final double K_P = 0.03;
        public static final double MANUAL_INTAKE_MULTIPLIER = 0.4;
        public static final int CONE_COLLECTED_VALUE = 5;
        public static final int TICKS_BEFORE_FINISHED = 3;
        public static final int CONE_HOLDING_AMPS = 8;

        public static final double INTAKE_HOLDING = .025;
        public static final double INTAKE_HOLDING_SPEED = INTAKE_HOLDING / WHEEL_CIRCUMFERENCE * WHEEL_REVS_PER_SEC_TO_VELOCITY;
    }
    public class JoystickPorts {

        public static final int LEFT_JOY_STICK_PORT = 0;
        public static final int RIGHT_JOY_STICK_PORT = 1;
        public static final int BUTTON_STICK_PORT = 2;
    }

    public class OIButtons {

        //slow mode driving
        public static final int DRIVE_SLOW = 1;

        //top row
        public static final int ENABLE_MANUAL =8;

        public static final int MANUAL_COLLECTOR_UP=6;

        public static final int MANUAL_COLLECTOR_DOWN=7;

        public static final int RESET_POSITION=1;

        public static final int WRIST_UP=5;

        public static final int WRIST_DOWN=4;
        public static final int MANUAL_SHOULDER_UP=2;

        public static final int MANUAL_SHOULDER_DOWN=3;

        public static final int OTHER_LIGHTS_UP=20;

        public static final int OTHER_LIGHTS_DOWN=12;

        //middle row

        public static final int SHOOT_CUBE=18;

        public static final int SCORE_LOW=17;

        public static final int SCORE_MID=24;

        public static final int STOW=13;

        public static final int COLLECT_GROUND=12;

        public static final int COLLECT_SHELF=11;

        public static final int INTAKE=23;

        public static final int EXPEL=22;

        //bottom row

        public static final int CONE_TOGGLE=10;

        public static final int LIGHTS_UP=9;

        public static final int LIGHTS_DOWN=16;



    }

    public static class Autos{

        public static class FirstAuto{
            public static Trajectory trajectoryOne= TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.feetToMeters(6.33), Units.feetToMeters(23), new Rotation2d(Math.PI)),
                    List.of(new Translation2d(Units.feetToMeters(6.33-3),Units.feetToMeters(23-.25))),
                    new Pose2d(Units.feetToMeters(6.33-10.5), Units.feetToMeters(23-.25), new Rotation2d(Math.PI)),
                    frc.robot.subsystems.Drivebase.GetDrivebase().config);

            //pickup
            public static Trajectory trajectoryTwo= TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.feetToMeters(6.33+10.5), 23.25, new Rotation2d(Math.PI)), List.of(new Translation2d(Units.feetToMeters(6.33+3),Units.feetToMeters(23.25))),
                    new Pose2d(Units.feetToMeters(6.33), Units.feetToMeters(23), new Rotation2d(0)), frc.robot.subsystems.Drivebase.GetDrivebase().config);

            //place second peice
            public static Trajectory trajectoryThree= TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.feetToMeters(6.33), Units.feetToMeters(23), new Rotation2d(0)), List.of(new Translation2d(Units.feetToMeters(7),Units.feetToMeters(26.6-7.33))),
                    new Pose2d(Units.feetToMeters(13), Units.feetToMeters(26.6-9.33), new Rotation2d(-Math.PI/2)), frc.robot.subsystems.Drivebase.GetDrivebase().config);

        }

        public static class testAuto
        {
            public static Trajectory trajectoryOne= TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d(0/2)),
                    List.of(new Translation2d(Units.feetToMeters(5), Units.feetToMeters(0))),
                    new Pose2d(Units.feetToMeters(10), Units.feetToMeters(0), new Rotation2d(0/2)),
                    frc.robot.subsystems.Drivebase.GetDrivebase().config);
        }

    }


    public class Drivebase {

        public static final double kMaxSpeedMetersPerSecond=.25;//was 3.47472
        public static final double kMaxAccelerationMetersPerSecondSquared=2.0792;//was 24.0792

        public static final double SLOW_MODE_RATIO = .165;
        public static final double GEAR_RATIO = 10.71;

        public static final double kTrackwidthMeters=.38;
        public static final double FEET_PER_METER=3.28084;
        public static final int TICKS_PER_ROTATION=2048;
        public static final double WHEEL_DIAMETER = 0.5;
        public static final double DISTANCE_KP = 0.05;
        public static final double ROTATE_KP = 0.0026/* 16,17,18 */;
        public static final double ROTATE_KF = 0.04;
        public static final double ANGLE_KP = 0.003;
        public static final double ANGLE_KD = 0.005;
        public static final double DRIVEBASE_KF = 0.08;
        public static final double THRESHOLD_ROTATE = 2.5;
        public static final int DRIVE_OUT_OF_COMMUNITY = -9;
        public static final double TURN_THROTTLE_MULTIPLIER = 0.7;
        public static final double ARCADE_DRIVE_KP = 0.007;
        public static final double ARCADE_DRIVE_KD = 0;
        public static final double ARCADE_DRIVE_LEFTX_DEADBAND = 0.01;

        public static final int DRIVE_OUT_OF_COMMUINITY = -9;

        public static final double VOLTAGE_TO_DISTANCE_SENSOR=1;
        public static final double MAXIMUM_BALANCE_DISTANCE_FROM_GROUND_FRONT=100;

        public static final double MAXIMUM_BALANCE_DISTANCE_FROM_GROUND_BACK=800;

        //set these
        public static final int FRONT_RANGE_SENSOR_OUTPUT_CHANNEL=4;

        public static final int BACK_RANGE_SENSOR_OUTPUT_CHANNEL=5;

        public static final int FRONT_RANGE_SENSOR_INPUT_CHANNEL=0;

        public static final int BACK_RANGE_SENSOR_INPUT_CHANNEL=1;
        public static final double ARCADE_DRIVE_MAX_DEGREES_PER_SECOND = 250;
        public static final int EXECUTES_PER_SECOND = 50;
        public static final double  WAIT_TIME_FOR_GYRO_CALIBRATION = 3;
        public static final double HEADING_TOO_BIG = 20;
        public static final double MAX_DRIVE_DISTANCE_SPEED = 0.3;

        public static final double FEEDFORWARD_KS=.0015;

        public static final double FEEDFORWARD_KV=.238;

        public static final double FEEDFORWARD_KA=0.00;
    }

    public class Arm {
        public static final double MIN_ANGLE = Arm.SHOULDER_RESTING_ANGLE;
        public static final double MAX_ANGLE = 25;
        public static final double MIN_PEAK = .2;
        public static final double MAX_PEAK = .7;

        public static final double SHOULDER_MANUAL_SPEED=0.12;

        public static final double SHOULDER_TICKS_TO_DEGREES = ((1.0 / Falcon500.TICKS_PER_REV) / Arm.SHOULDER_GEAR_RATIO) * 360;
        public static final double WRIST_TICKS_TO_DEGREES = ((1.0 / Falcon500.TICKS_PER_REV) / Arm.WRIST_GEAR_RATIO) * 360;
        public static final double SHOULDER_KP = 0.013; //0.064
        public static final double SHOULDER_KI = 0;
        public static final double SHOULDER_KF = -0.041;
        public static final double SHOULDER_KP_AUTO = 0.016;
        public static final double SHOULDER_PEAK_OUTPUT = 0.7;
        public static final double SHOULDER_PEAK_OUTPUT_AUTO = 0.75;
        public static final double SHOULDER_TOLERANCE = 2;
        public static final double SHOULDER_GEAR_RATIO = 137.4;
        public static final int SHOULDER_SWAP_ANGLE = 0;
        public static final int SHOULDER_SWAP_ANGLE_ADDITION = 0;
        public static final double SHOULDER_RESTING_ANGLE = -128.59537049672488;
        public static final double SHOULDER_ANGLE_UPDATE = 5;
        public static final int SHOULDER_MOTOR_ID = 5;
        public static final int SHOULDER_LIMIT_SWITCH_FRONT = 0;
        public static final int SHOULDER_LIMIT_SWITCH_BACK = 1;
        public static final int WRIST_LIMIT_SWITCH = 2;

        public static final int MAX_WRIST_ROTATION = 235;
        public static final double WRIST_TOLERANCE = 2;
        public static final double WRIST_PARALLEL_WITH_SHOULDER = 164.05;
        public static final double WRIST_POWER=.15;
        public static final int WRIST_MOTOR_DEVICE_NUMBER = 6;
        public static final double WRIST_GEAR_RATIO = 176;
        public static final double WRIST_RESTING_ANGLE = 0;
        public static final double WRIST_LIMIT_ANGLE = 0;
        public static final double WRIST_PEAK_OUTPUT = 1;
        public static final double WRIST_KP = 0.05;
        public static final double WRIST_KI = 0;
        public static final double WRIST_KD = 0;
        public static final double WRIST_KF = 0;
    }

    public enum ArmPose {
        FLOOR_WEIRD (-90,220, 1, false),
        FLOOR_NORMAL  (-128.59537049672488, 100.70434, -1, false),
        FLOOR_CUBE ( -128.59537049672488, 115.25, -1, false),
        FLOOR_CONE ( -128.59537049672488, 112.75, -1, false),
        HIGH_CUBE (41.57, 100, -1, false),
        SUBSTATION_CUBE ( 43.68152, 174.73779, 1, false),
        SUBSTATION_CONE ( 42.56579,  155.46118, 1, true),
        SCORE_MID_CUBE ( 56.56579, 160.46118, 1,false),
        SCORE_MID_CONE (44.56579,  150.46118, 1,false),
        STOW (-128.59537049672488, 15.531, -1,false),//TODO: stow is wierd
        STOW_AUTO (-128.59537049672488, 15.531, -1.15,false),
        HIGH_CUBE_AUTO (41.57, 96, -1.15,false);

        public final double shoulderAngle;
        public final double wristAngle;
        public final double ConeIntakeDirection;
        public final boolean shouldAutoStow;
        public double ConeIntake() {
            return ConeIntakeDirection;
        }
        public double CubeIntake() {
            return -ConeIntakeDirection;
        }
        public double CubeExpel() {
            return ConeIntakeDirection;
        }
        public double ConeExpel() {
            return -ConeIntakeDirection;
        }
        ArmPose(double shoulderAngle, double wristAngle, double ConeIntakeDirection, boolean shouldAutoStow) {
            this.shouldAutoStow = shouldAutoStow;
            this.ConeIntakeDirection = ConeIntakeDirection;
            this.shoulderAngle = shoulderAngle;
            this.wristAngle = wristAngle;
        }

    }

    public class ArmPos
    {
        public static final double FLOOR_CUBE_PICKUP_WRIST = 112.75;
        public static final double CARRY_WRIST = 15.531;
        public static final double SCORE_CONE_WEIRD_SHOULDER = -90;
    }

    public class Lights
    {
        public static final int LIGHT_BIT_0 = 6;
        public static final int LIGHT_BIT_1 = 7;
        public static final int LIGHT_BIT_2 = 8;
        public static final int LIGHT_BIT_3 = 9;

        //colors
        public static final int BLANK = 0;
        public static final int CUBE = 1;
        public static final int CONE = 2;
        public static final int BLUE_RED = 3;
        public static final int BLUE_WITH_WHITE = 4;
        public static final int RED_WITH_WHITE = 5;
        public static final int PARTY = 6;
        public static final int HARD_RIGHT = 7;
        public static final int MEDIUM_RIGHT = 8;
        public static final int SLIGHT_RIGHT = 9;
        public static final int CENTER = 10;
        public static final int SLIGHT_LEFT = 11;
        public static final int MEDIUM_LEFT = 12;
        public static final int HARD_LEFT = 13;
        public static final int CYLON = 14;
        public static final int RAINBOW_CHASE = 15;

    }

    public class Falcon500 {
        public static final int TICKS_PER_REV = 2048;
    }

    public class FourMotorFalcon500 {
        public static final int LEFT_MOTOR_1_DEVICE_NUMBER = 1;
        public static final int RIGHT_MOTOR_1_DEVICE_NUMBER = 4;
        public static final int LEFT_MOTOR_2_DEVICE_NUMBER = 2;
        public static final int RIGHT_MOTOR_2_DEVICE_NUMBER = 3;
    }

// MULTI-DRIVEBASE
        public class Robot2022 {

            //Motor Ports
            public static final int LEFT_MOTOR_1 = 4;
            public static final int RIGHT_MOTOR_1 = 1;
            public static final int LEFT_MOTOR_2 = 3;
            public static final int RIGHT_MOTOR_2 = 2;
            public static final int LEFT_DIRECTION = 1; //Wheels turn in correct direction
            public static final int RIGHT_DIRECTION = -1;

            //Encoders
            public static final double GEAR_RATIO = 10.71;
            public static final double WHEEL_DIAMETER = .5; //feet
            public static final int TICKS_PER_MOTOR_REV = 2048;
            public static final double TICKS_PER_FOOT = (TICKS_PER_MOTOR_REV * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
        }

        public class Robot2020 {

            //Motor Ports
            public static final int LEFT_MOTOR_1 = 1;
            public static final int RIGHT_MOTOR_1 = 13;
            public static final int LEFT_MOTOR_2 = 2;
            public static final int RIGHT_MOTOR_2 = 12;

            public static final int LEFT_DIRECTION = 1;  //Wheels turn in correct direction
            public static final int RIGHT_DIRECTION = 1;

            //Encoders
            public static final double GEAR_RATIO = 55;
            public static final double WHEEL_DIAMETER = .5; //feet
            public static final int TICKS_PER_MOTOR_REV = 496;
            public static final double TICKS_PER_FOOT =
                    (TICKS_PER_MOTOR_REV * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
        }

        public class Wobbles /* Same motor ports as 2023 */ {

            //Motor Ports
            public static final int LEFT_MOTOR_1 = 3;
            public static final int RIGHT_MOTOR_1 = 1;
            public static final int LEFT_MOTOR_2 = 4;
            public static final int RIGHT_MOTOR_2 = 2;

            public static final int LEFT_DIRECTION = 1; //Wheels turn in correct direction
            public static final int RIGHT_DIRECTION = -1;

            //Encoders
            public static final double GEAR_RATIO = 10.71;
            public static final double WHEEL_DIAMETER = .5; //feet
            public static final int TICKS_PER_MOTOR_REV = 2048;
            public static final double TICKS_PER_FOOT =
                    (TICKS_PER_MOTOR_REV * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
        }
    }
