package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class Constants extends CommandBase {

    public static final double MILISECOND_PER_SECOND=.1;

    public class JoystickPorts {

        public static final int LEFT_JOY_STICK_PORT = 0;
        public static final int RIGHT_JOY_STICK_PORT = 1;
        public static final int BUTTON_STICK_PORT = 2;
    }

    public class Drivebase {
        public static final double GEAR_RATIO = 10.71;
        public static final double WHEEL_DIAMETER = 0.5;
        public static final double DISTANCE_KP = 0.05;
        public static final double ROTATE_KP = 0.001;
        public static final double ROTATE_KF = 0.04;

        public static final double KS=.093822;
        public static final double KV=.57874;
        public static final double KA=.082382;

        public static final double TRACK_WIDTH_METERS=0.381;




        public static final double MAX_SPEED_METERS_PER_SECOND=.1;                          //was 3.47472

        public static final double DRIVEBASE_KP = .015;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED=.2792;     //was 24.0792





        public static final double TRAJECTORY_MAX_VELOCITY=MAX_SPEED_METERS_PER_SECOND;

        public static final double TRAJECTORY_MAX_ACCELERATION=MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;

        public static final double ANGLE_KP = 0.018;
        public static final double DRIVEBASE_KF = 0.08;
        public static final int THRESHOLD_ROTATE = 3;
        public static final int DRIVE_OUT_OF_COMMUINITY = -9;


    }

    public class Arm
    {
        public static final double COLLECTOR_MOTOR_1_KP = 0.5;
        public static final int GEAR_RATIO = 200;
    }
    public class Falcon500 {
        public static final int TICKS_PER_REV = 2046;
    }

    public class FourMotorFalcon500 {
        public static final int LEFT_MOTOR_1_DEVICE_NUMBER = 1;
        public static final int RIGHT_MOTOR_1_DEVICE_NUMBER = 4;
        public static final int LEFT_MOTOR_2_DEVICE_NUMBER = 2;
        public static final int RIGHT_MOTOR_2_DEVICE_NUMBER = 3;
    }


    public class MultiDrivebase {
        public class Robot2022 {

            //Motor Ports
            public static final int LEFT_MOTOR_1 = 4;
            public static final int RIGHT_MOTOR_1 = 1;
            public static final int LEFT_MOTOR_2 = 3;
            public static final int RIGHT_MOTOR_2 = 2;

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

            //Encoders
            public static final double GEAR_RATIO = 55;
            public static final double WHEEL_DIAMETER = .5; //feet
            public static final int TICKS_PER_MOTOR_REV = 496;
            public static final double TICKS_PER_FOOT =
                    (TICKS_PER_MOTOR_REV * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
        }

        public class Wobbles {

            //Motor Ports
            public static final int LEFT_MOTOR_1 = 3;
            public static final int RIGHT_MOTOR_1 = 1;
            public static final int LEFT_MOTOR_2 = 4;
            public static final int RIGHT_MOTOR_2 = 2;

            //Encoders
            public static final double GEAR_RATIO = 10.71;
            public static final double WHEEL_DIAMETER = .5; //feet
            public static final int TICKS_PER_MOTOR_REV = 2048;
            public static final double TICKS_PER_FOOT =
                    (TICKS_PER_MOTOR_REV * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);


            public static final double FEET_PER_METER=3.28084;

            public static final double TICKS_PER_METER=TICKS_PER_FOOT*FEET_PER_METER;
        }
    }
}
