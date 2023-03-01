package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class Constants extends CommandBase {
    public static class Collector {
        public static final int MOTOR_ID = 7;


        public static final double  GEAR_RATIO = 2;
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * 2;
        public static final double WHEEL_REVS_PER_SEC_TO_VELOCITY = 8050;
        public static final int CUBE_BREAK_1_PORT = 0;
        public static final int CUBE_BREAK_2_PORT = 1;
        public static final double INTAKE_SPEED = -8; //inches per second
        public static final double INTAKE_MOTOR_SPEED =
                INTAKE_SPEED / WHEEL_CIRCUMFERENCE * WHEEL_REVS_PER_SEC_TO_VELOCITY;
        public static final double EXPEL_SPEED = 8; // inches per second
        public static final double EXPEL_MOTOR_SPEED =
                EXPEL_SPEED / WHEEL_CIRCUMFERENCE * WHEEL_REVS_PER_SEC_TO_VELOCITY;
        public static final double K_P = 0.03;
    }
    public class JoystickPorts {

        public static final int LEFT_JOY_STICK_PORT = 0;
        public static final int RIGHT_JOY_STICK_PORT = 1;
        public static final int BUTTON_STICK_PORT = 2;
    }

    public class OIButtons {
        public static final int ARM_UP_BUTTON = 12;
        public static final int ARM_DOWN_BUTTON = 15;
        public static final int WRIST_UP_BUTTON=4;
        public static final int WRIST_DOWN_BUTTON=2;
        public static final int INTAKE_BUTTON = 13; // 13 on comp robot
        public static final int EXPEL_BUTTON = 9; // 9 on comp robot

    }

    public class Drivebase {
        public static final double GEAR_RATIO = 10.71;
        public static final double WHEEL_DIAMETER = 0.5;
        public static final double DISTANCE_KP = 0.05;
        public static final double ROTATE_KP = 0.001;
        public static final double ROTATE_KF = 0.04;
        public static final double ANGLE_KP = 0.003;
        public static final double DRIVEBASE_KF = 0.08;
        public static final double THRESHOLD_ROTATE = .5;
        public static final int DRIVE_OUT_OF_COMMUINITY = -9;
    }

    public class Arm {
        public static final double SHOULDER_TICKS_TO_DEGREES = ((1.0 / Constants.Falcon500.TICKS_PER_REV) / Constants.Arm.SHOULDER_GEAR_RATIO) * 360;
        public static final double SHOULDER_KP = 0.064;
        public static final double SHOULDER_KI = 0;
        public static final double SHOULDER_KF = -0.0385;
        public static final double SHOULDER_PEAK_OUTPUT = 0.3;
        public static final double SHOULDER_TOLERANCE = 2;
        public static final int SHOULDER_GEAR_RATIO = 128;
        public static final int SHOULDER_SWAP_ANGLE = 0;
        public static final int SHOULDER_SWAP_ANGLE_ADDITION = 0;
        public static final double SHOULDER_RESTING_ANGLE = -96.6;
        public static final double SHOULDER_ANGLE_UPDATE = 0.5;
        public static final int SHOULDER_MOTOR_ID = 5;
        public static final int SHOULDER_LIMIT_SWITCH_FRONT = 0;
        public static final int SHOULDER_LIMIT_SWITCH_BACK = 1;
        public static final int WRIST_MOTOR_DEVICE_NUMBER = 6;
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
