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

        //center on game piece teleop assist
        public static final int CENTER_ON_PIECE = 2;

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

    public class Drivebase {

        public static final double SLOW_MODE_RATIO = .165;
        public static final double GEAR_RATIO = 10.71;
        public static final double WHEEL_DIAMETER = 0.5;
        public static final double DISTANCE_KP = 0.05;
        public static final double ROTATE_KP = 0.0026/* 16,17,18 */;
        public static final double ROTATE_KF = 0.04;
        public static final double ANGLE_KP = 0.003;
        public static final double ANGLE_KD = 0.005;
        public static final double DRIVEBASE_KF = 0.08;
        public static final double THRESHOLD_ROTATE = 3;
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

        public static final double BASE_DRIVE_TO_CONE_SPEED = -0.18;
        public static final double DRIVE_TO_CONE_KP = 0.005;

        public static final double LIMELIGHT_MAX_CONE_AREA = 150000; //figure this out

        public static final double LIMELIGHT_CAMERA_PIXEL_WIDTH = 320;
        public static final double LIMELIGHT_CAMERA_PIXEL_HEIGHT = 240;
        public static final int ROLLING_AVERAGE_LENGTH = 5;

    }

    public class Arm {
        public static final double MIN_ANGLE = Arm.SHOULDER_RESTING_ANGLE;
        public static final double MAX_ANGLE = 25;
        public static final double MIN_PEAK = .2;
        public static final double MAX_PEAK = .7;

        public static final double SHOULDER_MANUAL_SPEED=0.12;

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
        public static final int MAX_WRIST_ROTATION = 235;
        public static final double WRIST_TOLERANCE = 2;
        public static final double WRIST_PARALLEL_WITH_SHOULDER = 164.05;
        public static final double WRIST_POWER=.15;
        public static final int WRIST_MOTOR_DEVICE_NUMBER = 6;
        public static final double WRIST_GEAR_RATIO = 176;
        public static final double WRIST_RESTING_ANGLE = 0;
        public static final double WRIST_LIMIT_ANGLE = 0;
        public static final double WRIST_PEAK_OUTPUT = 0.6;
        public static final double WRIST_KP = 0.05;
        public static final double WRIST_KI = 0;
        public static final double WRIST_KD = 0;
        public static final double WRIST_KF = 0;
    }

    public class ArmPos
    {

        public static final double PLAYER_CONE_PICKUP_SHOULDER = 40.56579;
        public static final double PLAYER_CONE_PICKUP_WRIST = 155.46118;
        public static final double SCORE_CONE_MID_SHOULDER = 44.56579;
        public static final double SCORE_CUBE_MID_SHOULDER = 56.56579;
        public static final double SCORE_CONE_MID_WRIST = 150.46118;
        public static final double SCORE_CUBE_MID_WRIST = 160.46118;
        public static final double PLAYER_CUBE_PICKUP_SHOULDER = 43.68152;
        public static final double PLAYER_CUBE_PICKUP_WRIST = 174.73779;
        public static final double FLOOR_CUBE_PICKUP_SHOULDER = Arm.SHOULDER_RESTING_ANGLE;
        public static final double FLOOR_CUBE_PICKUP_WRIST = 112.75;
        public static final double FLOOR_NORMAL_SCORE_SHOULDER = Arm.SHOULDER_RESTING_ANGLE;
        public static final double FLOOR_NORMAL_SCORE_WRIST = 100.70434;
        public static final double CARRY_SHOULDER = Arm.SHOULDER_RESTING_ANGLE;
        public static final double CARRY_WRIST = 15.531;
        public static final double CONE_FLOOR_PICKUP_SHOULDER = Arm.SHOULDER_RESTING_ANGLE;
        public static final double CONE_FLOOR_PICKUP_WRIST = 105.05419;
        public static final int WRIST_GEAR_RATIO = 96;
        //public static final double SCORE_CUBE_HIGH_SHOULDER = 45;// scores high cube with wrist up.
        //public static final double SCORE_CUBE_HIGH_WRIST = 225; // scores high cube with wrist up.

        //The following two constants work well for scoring high with a cube with the wrist down.
        public static final double SCORE_CUBE_HIGH_SHOULDER = 39.57;
        public static final double SCORE_CUBE_HIGH_WRIST = 100;
        public static final double SCORE_CONE_HIGH_SHOULDER = 30;
        public static final double SCORE_CONE_HIGH_WRIST = 230;
        public static final double SCORE_CONE_WEIRD_SHOULDER = -90;
        public static final double SCORE_CODE_WEIRD_WRIST = 220;
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
