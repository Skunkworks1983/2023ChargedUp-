package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class SetArmPositionCommand extends CommandBase
{

    private final Arm arm;
    private final double shoulderAngleSetpoint;
    private final double wristAngleSetpoint;

    public SetArmPositionCommand(double shoulderAngleSetpoint, double wristAngleSetpoint)
    {
        this.arm = Arm.getInstance();
        this.shoulderAngleSetpoint = shoulderAngleSetpoint;
        this.wristAngleSetpoint = wristAngleSetpoint;
        addRequirements(arm);
    }

    @Override
    public void initialize()
    {
        System.out.println("set arm pos with wristAngleSetpoint: " + wristAngleSetpoint + " and shoulderAngleSetpoint: " + shoulderAngleSetpoint);
        arm.setWristAnglePosition(wristAngleSetpoint);
        arm.setShoulderAnglePosition(shoulderAngleSetpoint);
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        //return Math.abs(arm.ShoulderMotor.getClosedLoopError() * Constants.Arm.SHOULDER_TICKS_TO_DEGREES) < Constants.Arm.SHOULDER_TOLERANCE &&
        //       Math.abs(arm.WristMotor.getClosedLoopError() * Constants.Arm.WRIST_TICKS_TO_DEGREES) < Constants.Arm.WRIST_TOLERANCE;
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        if(interrupted)
        {
            System.out.println("SetArmPositionCommand Ending, interrupted");
        }
        else
        {
            System.out.println("SetArmPositionCommand Ending");
        }
    }
}
