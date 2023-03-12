package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class Constants extends CommandBase {
    public static class Collector {
        public static final int MOTOR_ID = 7;

        public static final double  GEAR_RATIO = 2;
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * 2;
        public static final double WHEEL_REVS_PER_SEC_TO_VELOCITY = 8050;
        public static final int CUBE_BREAK_1_PORT = 2;
        public static final int CUBE_BREAK_2_PORT = 3;

        public static final double CONE_COLLECT_AMP_THRESHOLD = 22;
        public static final double INTAKE_SPEED = 10; //inches per second
        public static final double INTAKE_MOTOR_SPEED =
                INTAKE_SPEED / WHEEL_CIRCUMFERENCE * WHEEL_REVS_PER_SEC_TO_VELOCITY;
        public static final double EXPEL_SPEED = 10; // inches per second
        public static final double EXPEL_MOTOR_SPEED =
                EXPEL_SPEED / WHEEL_CIRCUMFERENCE * WHEEL_REVS_PER_SEC_TO_VELOCITY;
        public static final double K_P = 0.03;
        public static final double MANUAL_INTAKE_MULTIPLIER = 0.4;
        public static final int CONE_COLLECTED_VALUE = 3;

    }
    public class JoystickPorts {

        public static final int LEFT_JOY_STICK_PORT = 0;
        public static final int RIGHT_JOY_STICK_PORT = 1;
        public static final int BUTTON_STICK_PORT = 2;
    }

    public class OIButtons {
        public static final int ARM_UP_BUTTON = 12;
        public static final int ARM_DOWN_BUTTON = 15;
        public static final int WRIST_UP_BUTTON = 4; // NEEDS CHANGING BEFORE COMMITTING
        public static final int WRIST_DOWN_BUTTON=2;
        public static final int INTAKE_BUTTON = 13; // 13 on comp robot, 5 on  single stick
        public static final int EXPEL_BUTTON = 9; // 9 on comp robot, 6 on single stick
        public static final int CONE_TOGGLE = 10;
        public static final int MANUAL_TOGGLE = 1;


    }

    public class Drivebase {
        public static final double GEAR_RATIO = 10.71;
        public static final double WHEEL_DIAMETER = 0.5;
        public static final double DISTANCE_KP = 0.05;
        public static final double ROTATE_KP = 0.0026/* 16,17,18 */;
        public static final double ROTATE_KF = 0.04;
        public static final double ANGLE_KP = 0.003;
        public static final double DRIVEBASE_KF = 0.08;
        public static final double THRESHOLD_ROTATE = .5;
        public static final int DRIVE_OUT_OF_COMMUNITY = -9;
        public static final double ARCADE_DRIVE_KP = 0.015;
        public static final double ARCADE_DRIVE_KD = 0;
        public static final int DRIVE_OUT_OF_COMMUINITY = -9;

        public static final double VOLTAGE_TO_DISTANCE_SENSOR=1;
        public static final double MAXIMUM_BALANCE_DISTANCE_FROM_GROUND_FRONT=100;

        public static final double MAXIMUM_BALANCE_DISTANCE_FROM_GROUND_BACK=800;

        //set these
        public static final int FRONT_RANGE_SENSOR_OUTPUT_CHANNEL=4;

        public static final int BACK_RANGE_SENSOR_OUTPUT_CHANNEL=5;

        public static final int FRONT_RANGE_SENSOR_INPUT_CHANNEL=0;

        public static final int BACK_RANGE_SENSOR_INPUT_CHANNEL=1;
        public static final double ARCADE_DRIVE_MAX_DEGREES_PER_SECOND = 150;
        public static final int EXECUTES_PER_SECOND = 50;
        public static final double  WAIT_TIME_FOR_GYRO_CALIBRATION = 3;
        public static final double MAX_DRIVE_DISTANCE_SPEED = 0.3;
    }

    public class Arm {
        public static final double MIN_ANGLE = Arm.SHOULDER_RESTING_ANGLE;
        public static final double MAX_ANGLE = 25;
        public static final double MIN_PEAK = .3;
        public static final double MAX_PEAK = .65;

        public static final double SHOULDER_TICKS_TO_DEGREES = ((1.0 / Falcon500.TICKS_PER_REV) / Arm.SHOULDER_GEAR_RATIO) * 360;
        public static final double WRIST_TICKS_TO_DEGREES = ((1.0 / Falcon500.TICKS_PER_REV) / Arm.WRIST_GEAR_RATIO) * 360;
        public static final double SHOULDER_KP = 0.06; //0.064
        public static final double SHOULDER_KI = 0;
        public static final double SHOULDER_KF = -0.041;
        public static final double SHOULDER_PEAK_OUTPUT = 0.45;
        public static final double SHOULDER_TOLERANCE = 2;
        public static final double SHOULDER_GEAR_RATIO = 137.4;
        public static final int SHOULDER_SWAP_ANGLE = 0;
        public static final int SHOULDER_SWAP_ANGLE_ADDITION = 0;
        public static final double SHOULDER_RESTING_ANGLE = -128.59537049672488;
        public static final double SHOULDER_ANGLE_UPDATE = 1.5;
        public static final int SHOULDER_MOTOR_ID = 5;
        public static final int SHOULDER_LIMIT_SWITCH_FRONT = 0;
        public static final int SHOULDER_LIMIT_SWITCH_BACK = 1;
        public static final int WRIST_LIMIT_SWITCH = 2;
        //once shoulder is passed this angle, wrist can go anywhere.
        public static final double SHOULDER_SAFE_WRIST_ANGLE = SHOULDER_RESTING_ANGLE;
        public static final int MAX_WRIST_ROTATION = 180;
        public static final double WRIST_TOLERANCE = 2;
        public static final int WRIST_MOTOR_DEVICE_NUMBER = 6;
        public static final double WRIST_GEAR_RATIO = 176;
        public static final double WRIST_RESTING_ANGLE = 0;
        public static final double WRIST_PEAK_OUTPUT = 0.4;
        public static final double WRIST_KP = 0.05;
        public static final double WRIST_KI = 0;
        public static final double WRIST_KD = 0;
        public static final double WRIST_KF = 0;
    }

    public class ArmPos
    {
        public static final double SCORE_CONE_MID_SHOULDER = 51.23676;
        public static final double SCORE_CONE_MID_WRIST = 160.13574;
        public static final double PLAYER_CONE_PICKUP_SHOULDER = 39.56579;
        public static final double PLAYER_CONE_PICKUP_WRIST = 156.46118;
        public static final double PLAYER_CUBE_PICKUP_SHOULDER = 46.68152;
        public static final double PLAYER_CUBE_PICKUP_WRIST = 171.73779;
        public static final double FLOOR_CUBE_PICKUP_SHOULDER = Arm.SHOULDER_RESTING_ANGLE;
        public static final double FLOOR_CUBE_PICKUP_WRIST = 114.25;
        public static final double FLOOR_NORMAL_SCORE_SHOULDER = Arm.SHOULDER_RESTING_ANGLE;
        public static final double FLOOR_NORMAL_SCORE_WRIST = 100.70434;
        public static final double CARRY_SHOULDER = Arm.SHOULDER_RESTING_ANGLE;
        public static final double CARRY_WRIST = 15.531;
        public static final double CONE_FLOOR_PICKUP_SHOULDER = Arm.SHOULDER_RESTING_ANGLE;
        public static final double CONE_FLOOR_PICKUP_WRIST = 103.05419;
        public static final int WRIST_GEAR_RATIO = 96;
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
