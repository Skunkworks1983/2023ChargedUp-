package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import org.opencv.video.TrackerDaSiamRPN;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;
import java.util.function.BiConsumer;


public class SmartDriveCommand extends CommandBase {
    BiConsumer<Double, Double> metersPerSecond;
    Trajectory trajectory;
    RamseteCommand ramseteCommand;
    double prevTime;
    Timer timer;
    public SmartDriveCommand(Trajectory trajectory) {

        timer = new Timer();
        timer.start();

        this.metersPerSecond=(leftSpeedSetpoint, rightSpeedSetpoint) -> {

            var desiredPose = trajectory.sample(timer.get());


            ChassisSpeeds refChassisSpeeds = DriveBase.getInstance().ramseteController.calculate(DriveBase.getInstance().getPose(), desiredPose);
            DriveBase.getInstance().setSpeedChassis(refChassisSpeeds);//(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
        System.out.println((desiredPose.poseMeters.getX()-DriveBase.getInstance().getPose().getX())+","+(desiredPose.poseMeters.getY()-DriveBase.getInstance().getPose().getY()));
            SmartDashboard.putNumber("x error", desiredPose.poseMeters.getX()-DriveBase.getInstance().getPose().getX());
            SmartDashboard.putNumber("y error", desiredPose.poseMeters.getY()-DriveBase.getInstance().getPose().getY());
            SmartDashboard.putNumber("θ error", desiredPose.poseMeters.getRotation().getDegrees()-DriveBase.getInstance().getPose().getRotation().getDegrees());

        };
        this.trajectory=trajectory;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
        // Create config for trajectory

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(3, 0, new Rotation2d(0)),
                        // Pass config
                        DriveBase.getInstance().config);

        ramseteCommand =
                new RamseteCommand(trajectory,
                        DriveBase.getInstance()::GetCurrentPose,
                        DriveBase.getInstance().ramseteController,
                        new DifferentialDriveKinematics(DriveBase.getInstance().trackWidthMeters),
                        metersPerSecond);
        CommandScheduler.getInstance().schedule(ramseteCommand);

    }


    @Override
    public void execute() {

   }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        if(ramseteCommand.isFinished())System.out.println("is fineshed");
        return ramseteCommand.isFinished();

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("is fineshed");
    }
}
