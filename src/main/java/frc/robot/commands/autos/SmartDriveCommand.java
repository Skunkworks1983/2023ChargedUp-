package frc.robot.commands.autos;

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
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
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
            System.out.println(desiredPose.poseMeters.getX()+","+desiredPose.poseMeters.getY()+","+desiredPose.poseMeters.getRotation());
            ChassisSpeeds refChassisSpeeds = Drivebase.GetDrivebase().ramseteController.calculate(Drivebase.GetDrivebase().getPose(), desiredPose);
            Drivebase.GetDrivebase().setSpeedChassis(refChassisSpeeds);//(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
        };
        this.trajectory=trajectory;
        addRequirements();
    }

    @Override
    public void initialize() {
        // Create config for trajectory

        ramseteCommand =
                new RamseteCommand(trajectory,
                        Drivebase.GetDrivebase()::GetCurrentPose,
                        Drivebase.GetDrivebase().ramseteController,
                        new DifferentialDriveKinematics(Constants.Drivebase.kTrackwidthMeters),
                        metersPerSecond);
        CommandScheduler.getInstance().schedule(ramseteCommand);

    }


    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return ramseteCommand.isFinished();

    }

    @Override
    public void end(boolean interrupted) {

    }
}
