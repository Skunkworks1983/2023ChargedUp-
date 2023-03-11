package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Collector;


public class IntakeConeCollectorCommand extends CommandBase {
    public Collector collectorInstance;

    public IntakeConeCollectorCommand() {
        collectorInstance = Collector.getInstance();
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
        collectorInstance.Setspeed(-Constants.Collector.INTAKE_MOTOR_SPEED);
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished() {

        return false;

    }

    @Override
    public void end(boolean interrupted)
    {
        collectorInstance.Setspeed(0);
    }
}
