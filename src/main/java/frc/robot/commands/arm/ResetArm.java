package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;

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
        return arm.ShoulderMotor.getSensorCollection().isRevLimitSwitchClosed() == 1 && arm.WristMotor.getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    @Override
    public void end(boolean interrupted)
    {
        arm.setShoulderAnglePosition(Constants.Arm.SHOULDER_RESTING_ANGLE);
        arm.setWristAnglePosition(Constants.Arm.WRIST_RESTING_ANGLE);
    }
}
