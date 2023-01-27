package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class Constants extends CommandBase
{
    public class JoystickPorts
    {

        public static final int LEFT_JOY_STICK_PORT = 0;
        public static final int RIGHT_JOY_STICK_PORT = 1;
        public static final int BUTTON_STICK_PORT = 2;
    }
    public class OIButtons
    {
        public static final int TEST_BUTTON_ONE = 2;
        public static final int TEST_BUTTON_TWO = 3;
        public static final int TEST_BUTTON_THREE = 4;
        public static final int TEST_BUTTON_FOUR = 5;
    }

    public class Drivebase
    {
        public static final double GEAR_RATIO = 10.71;
        public static final double WHEEL_DIAMETER = 0.5;
        public static final double DISTANCE_KP = 0.05;
        public static final double ROTATE_KP = 0.001;
        public static final double ROTATE_KF = 0.04;
        public static final double ANGLE_KP = 0.018;
        public static final double DRIVEBASE_KF = 0.08;
        public static final int THRESHOLD_ROTATE = 3;
        public static final int DRIVE_OUT_OF_COMMUINITY = -9;
    }

    public class Falcon500
    {
        public static final int TICKS_PER_REV = 2046;
    }

    public class FourMotorFalcon500
    {
        public static final int LEFT_MOTOR_1_DEVICE_NUMBER = 1;
        public static final int RIGHT_MOTOR_1_DEVICE_NUMBER = 4;
        public static final int LEFT_MOTOR_2_DEVICE_NUMBER = 2;
        public static final int RIGHT_MOTOR_2_DEVICE_NUMBER = 3;
    }
}
