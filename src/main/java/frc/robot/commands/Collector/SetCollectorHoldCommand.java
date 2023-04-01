package frc.robot.commands.Collector;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;


public class SetCollectorHoldCommand extends CommandBase {
    private final Collector collector = Collector.getInstance();

    public SetCollectorHoldCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(collector);
    }

    @Override
    public void initialize() {
        System.out.println("Collector Hold Initialize");
        collector.Motor.set(TalonFXControlMode.Position, 0);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {

            System.out.println("Collector Hold Command Ended, interrupted");
        } else {
            System.out.println("Collector Hold Command Ended");
        }
    }
}
