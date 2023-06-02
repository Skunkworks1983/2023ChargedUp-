package frc.robot.commands.drivebase;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;


public class PoseEstimatorTerminateCommand extends CommandBase
{
    private final Drivebase drivebase;
    private final double xTerminate;
    
    public PoseEstimatorTerminateCommand(double XTerminate)
    {
        this.drivebase = Drivebase.GetDrivebase();
        this.xTerminate = XTerminate;
        addRequirements();
    }

    @Override
    public void initialize()
    {
        System.out.println("initialising terminate command");
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return Units.metersToFeet(drivebase.GetCurrentPose().getX()) >= xTerminate;
    }

    @Override
    public void end(boolean interrupted)
    {
        if (interrupted) {
            System.out.println("terminate command Ended, interrupted, x pos is: " + Units.metersToFeet(drivebase.GetCurrentPose().getX()));
        } else {
            System.out.println("terminate command Ended, x pos is: " + Units.metersToFeet(drivebase.GetCurrentPose().getX()));
        }
    }
}
