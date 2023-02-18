package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

public class DrivebaseConstants {
    public static final double GEAR_RATIO = 10.71;
    public static final double WHEEL_DIAMETER = 0.5;
    public static final double DISTANCE_KP = 0.05;
    public static final double ROTATE_KP = 0.001;
    public static final double ROTATE_KF = 0.04;

    public static final double TRACK_WIDTH_METERS=0.381;

    public static final double MAX_SPEED_METERS_PER_SECOND=1;                          //was 3.47472

    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED=2.0792;     //was 24.0792

    public static final double TRAJECTORY_MAX_VELOCITY=1;

    public static final double TRAJECTORY_MAX_ACCELERATION=1;

    public static final TrajectoryConstraint AUTO_VOLTAGE_CONSTRAINT= new MaxVelocityConstraint(MAX_SPEED_METERS_PER_SECOND);

    public static final DifferentialDriveKinematics kDriveKinematics= new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    public static final double ANGLE_KP = 0.018;
    public static final double DRIVEBASE_KF = 0.08;
    public static final int THRESHOLD_ROTATE = 3;
    public static final int DRIVE_OUT_OF_COMMUINITY = -9;
}
