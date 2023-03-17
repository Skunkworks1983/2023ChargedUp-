package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Collector;


public class CollectorPercentOutputCommand extends CommandBase {
double speed;
    Collector collector = Collector.getInstance();
    public CollectorPercentOutputCommand(double speed) {
        this.speed = speed;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(collector);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        collector.SetPercentOutput(speed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        collector.SetPercentOutput(0);
    }
}
