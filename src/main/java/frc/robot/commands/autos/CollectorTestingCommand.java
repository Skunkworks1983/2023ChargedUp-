package frc.robot.commands.autos;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.services.Oi;
import frc.robot.subsystems.Collector;


public class CollectorTestingCommand extends CommandBase {

    public CollectorTestingCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
        //Collector.getInstance().intake();
    }

    @Override
    public void execute() {
        Collector.getInstance().Motor.set(TalonFXControlMode.PercentOutput, Oi.Instance.getLeftY());
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