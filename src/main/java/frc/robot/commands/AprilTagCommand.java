package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Limelight;


public class AprilTagCommand extends CommandBase {

    public AprilTagCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        Limelight.UpdateTable();
        System.out.println("("+Limelight.GetInstance().poseData[0]+","+Limelight.GetInstance().poseData[1]+","+Limelight.GetInstance().poseData[2]+")"+":"+Limelight.GetInstance().currentTag);
    Drivebase.getInstance().position =
            new Pose3d(Limelight.GetInstance().poseData[0],
                    Limelight.GetInstance().poseData[1],
                    Limelight.GetInstance().poseData[2],
        new Rotation3d(Limelight.GetInstance().poseData[3],
                Limelight.GetInstance().poseData[4],
                Limelight.GetInstance().poseData[5]));
    }


    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }


    @Override
    public void end(boolean interrupted) {

    }
}
