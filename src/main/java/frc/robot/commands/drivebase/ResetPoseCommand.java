package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;


public class ResetPoseCommand extends Command {
    Pose2d pose;
    public ResetPoseCommand(Pose2d pose) {
        this.pose = pose;
        addRequirements();
    }

    @Override
    public void initialize() {
        Drivebase.GetDrivebase().setPose(this.pose);
        System.out.println("resetPose initialize");
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("resetPose ended");
    }
}
