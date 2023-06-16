package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class ResetArm extends CommandBase {

    Arm arm = Arm.getInstance();

    public ResetArm()
    {
        addRequirements(arm);
    }

    @Override
    public void initialize()
    {
        arm.SetPercentOutput(-(Constants.Arm.SHOULDER_PEAK_OUTPUT/2));
        arm.SetWristSpeed(-(Constants.Arm.WRIST_PEAK_OUTPUT/2));
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished()
    {
        return arm.isArmReset();

    }

    @Override
    public void end(boolean interrupted)
    {
        if(!interrupted) {
            arm.setShoulderSensorPosition(Constants.Arm.SHOULDER_RESTING_ANGLE / Constants.Arm.SHOULDER_TICKS_TO_DEGREES);
            arm.setWristSensorPosition(Constants.Arm.WRIST_LIMIT_ANGLE / Constants.Arm.WRIST_TICKS_TO_DEGREES);
            arm.setWristAnglePosition(Constants.Arm.WRIST_RESTING_ANGLE);
            arm.setShoulderAnglePosition(Constants.Arm.SHOULDER_RESTING_ANGLE);
            System.out.println("ENDED RESET ARM!!!!!!!");
        }
        else
        {
            System.out.println("ENDED RESET ARM!!!!!!! and it was interrupted");
        }
    }
}
