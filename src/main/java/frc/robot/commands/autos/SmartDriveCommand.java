package frc.robot.commands.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
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
    List <Translation2d> goThrough;
    Pose2d finalPose;
    boolean reversed;

    private DifferentialDriveWheelSpeeds prevSpeeds=new DifferentialDriveWheelSpeeds(0,0);

    public SmartDriveCommand(Trajectory trajectory) {

        this.metersPerSecond = (leftSpeedSetpoint, rightSpeedSetpoint) -> {
            Drivebase.GetDrivebase().setLeftMeters(Drivebase.GetDrivebase().metersToTicks(leftSpeedSetpoint));
            Drivebase.GetDrivebase().setRightMeters(Drivebase.GetDrivebase().metersToTicks(rightSpeedSetpoint));
        };

        this.trajectory=trajectory;
        addRequirements(Drivebase.GetDrivebase());
    }

    public SmartDriveCommand(List <Translation2d> goThrough,Pose2d finalPose,boolean reversed) {

        this.metersPerSecond = (leftSpeedSetpoint, rightSpeedSetpoint) -> {
            Drivebase.GetDrivebase().setLeftMeters(Drivebase.GetDrivebase().metersToTicks(leftSpeedSetpoint));
            Drivebase.GetDrivebase().setRightMeters(Drivebase.GetDrivebase().metersToTicks(rightSpeedSetpoint));
        };
        this.goThrough = goThrough;
        this.finalPose = finalPose;
        this.reversed = reversed;
        addRequirements(Drivebase.GetDrivebase());
    }

    @Override
    public void initialize()
    {
        if(this.trajectory== null) {
            if (reversed) {
                trajectory = TrajectoryGenerator.generateTrajectory(
                        Drivebase.GetDrivebase().GetCurrentPose(),
                        goThrough,
                        finalPose,
                        Drivebase.GetDrivebase().reversedConfig
                );
            } else {
                trajectory = TrajectoryGenerator.generateTrajectory(
                        Drivebase.GetDrivebase().GetCurrentPose(),
                        goThrough,
                        finalPose,
                        Drivebase.GetDrivebase().config
                );
            }
        }
        SmartDashboard.putData("should be",Drivebase.GetDrivebase().getField());
        Drivebase.GetDrivebase().getField().getObject("traj").setTrajectory(trajectory);
        // Create config for trajectory
        SmartDashboard.putData("should be",Drivebase.GetDrivebase().getField());
        Drivebase.GetDrivebase().getField().getObject("traj").setTrajectory(trajectory);

        ramseteCommand =
                new RamseteCommand(trajectory,
                        Drivebase.GetDrivebase()::GetCurrentPose,
                        Drivebase.GetDrivebase().ramseteController,
                        new DifferentialDriveKinematics(Constants.Drivebase.kTrackwidthMeters),
                        metersPerSecond);
        CommandScheduler.getInstance().schedule(ramseteCommand);
    System.out.println("initialized SmartDriveCommand");
    }


    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return ramseteCommand.isFinished();


    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ended SmartDriveCommand, interrupted:"+interrupted);
        Drivebase.GetDrivebase().runMotor(0,0);
        ramseteCommand.cancel();
    }
}
