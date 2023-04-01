package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;


public class ConditionalStowCommand extends CommandBase {
    private final Arm arm = Arm.getInstance();

    public ConditionalStowCommand() {

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        System.out.println("Conditional Stow Initialized");
        if(arm.getCurrentPose().shouldAutoStow) {
            arm.setCurrentPosition(Constants.ArmPose.STOW);
        }
        Constants.ArmPose pos = arm.getCurrentPose();
        double shoulderAngleSetpoint = pos.shoulderAngle;
        double wristAngleSetpoint = pos.wristAngle;
        arm.setWristAnglePosition(wristAngleSetpoint);
        arm.setShoulderAnglePosition(shoulderAngleSetpoint);

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {

            System.out.println("Conditional Stow Command Ended, interrupted");
        } else {
            System.out.println("Conditional Stow Command Ended");
        }
    }
}
