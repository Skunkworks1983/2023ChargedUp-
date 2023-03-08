package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;


public class IntakeConeManualCommand extends CommandBase {
    private Collector collectorInstance;
    private Arm armInstance;

    public IntakeConeManualCommand() {
        armInstance = Arm.getInstance();
        collectorInstance = Collector.getInstance();
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {

        if(armInstance.getShoulderAngle() < 0) {
            collectorInstance.Setspeed(-Constants.Collector.INTAKE_MOTOR_SPEED * Constants.Collector.MANUAL_INTAKE_MULTIPLIER);
        }
        else {
            collectorInstance.Setspeed(Constants.Collector.INTAKE_MOTOR_SPEED * Constants.Collector.MANUAL_INTAKE_MULTIPLIER);
        }



    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Colletor current", collectorInstance.GetCollectorCurrent());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        collectorInstance.Setspeed(0);
    }
}