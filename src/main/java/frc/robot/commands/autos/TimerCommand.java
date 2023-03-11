package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TimerCommand extends CommandBase {
    double seconds;
    Timer timer = new Timer();

    public TimerCommand(double seconds) {
        this.seconds = seconds;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()

        return timer.get() >= seconds;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
