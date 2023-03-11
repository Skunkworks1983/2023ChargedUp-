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
        addRequirements(collectorInstance);
    }

    @Override
    public void initialize() {
        System.out.println("Intake cone manual initialize");
    }

    @Override
    public void execute() {
        if(armInstance.getShoulderAngle() < 0) {
            collectorInstance.Setspeed(-Constants.Collector.INTAKE_MOTOR_SPEED * Constants.Collector.MANUAL_INTAKE_MULTIPLIER);
        }
        else {
            collectorInstance.Setspeed(Constants.Collector.INTAKE_MOTOR_SPEED * Constants.Collector.MANUAL_INTAKE_MULTIPLIER);
        }
        SmartDashboard.putNumber("Colletor current", collectorInstance.GetCollectorCurrent());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Intake cone manual end");
        collectorInstance.Setspeed(0);
    }
}
