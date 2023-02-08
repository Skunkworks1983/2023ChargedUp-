package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.multidrivebase.AbstractDrivebase;

public class Rotate90Degrees extends CommandBase
{
    private final Arm arm;

    public Rotate90Degrees(Arm arm)
    {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.arm = arm;
    }

    @Override
    public void initialize()
    {
        arm.setCollectorAnglePosition(90);
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        System.out.println("current pos: " + Math.abs(arm.getShoulderAngle()));
        if (Math.abs(arm.getShoulderAngle() - 90) < 3)
        {
            System.out.println("Ended");
            return true;
        }
        else
        {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted)
    {

    }
}
