package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Collector;


public class CollectorIntakeTeleopCommand extends CommandBase {
    public Collector collectorInstance;

    public CollectorIntakeTeleopCommand() {
        collectorInstance = Collector.getInstance();
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
        collectorInstance.Setspeed(Constants.Collector.INTAKE_MOTOR_SPEED);
    }

    @Override
    public void execute()
    {
        SmartDashboard.putNumber("power drawn: " , collectorInstance.GetCollectorCurrent());
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        collectorInstance.Setspeed(0);
    }
}
