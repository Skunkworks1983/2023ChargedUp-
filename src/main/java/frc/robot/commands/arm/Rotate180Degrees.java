package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Rotate180Degrees extends CommandBase
{
    private final Arm arm;

    public Rotate180Degrees(Arm arm)
    {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.arm = arm;
    }

    @Override
    public void initialize()
    {
        if((Math.abs(arm.getShoulderAngle()-180)) < 5)
        {
            arm.setShoulderAnglePosition(0);
        }
        else
        {
            arm.setShoulderAnglePosition(180);
        }

    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        System.out.println("current pos: " + Math.abs(arm.getShoulderAngle()));
        if (Math.abs(arm.getShoulderAngle() - 180) < 3)
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
