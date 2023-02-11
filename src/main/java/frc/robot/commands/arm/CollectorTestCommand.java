package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;


public class CollectorTestCommand extends CommandBase
{
     Collector collector;
    public CollectorTestCommand()
    {
        collector = Collector.getInstance();
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize()
    {
        collector.setCollectorSpeed(63*2048);
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
     collector.setCollectorSpeed(0);
    }
}
