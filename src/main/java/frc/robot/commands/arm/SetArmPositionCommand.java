package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class SetArmPositionCommand extends CommandBase
{

    private final Arm arm;
    private final double shoulderAngleSetpoint;
    private final double wristAngleSetpoint;
    
    private final Arm.PoseType pose;

    public SetArmPositionCommand(Arm.PoseType pose, double shoulderAngleSetpoint, double wristAngleSetpoint)
    {
        this.arm = Arm.getInstance();
        this.shoulderAngleSetpoint = shoulderAngleSetpoint;
        this.wristAngleSetpoint = wristAngleSetpoint;
        this.pose = pose;
        addRequirements(arm);
    }

    @Override
    public void initialize()
    {
        System.out.println("set arm pos with wristAngleSetpoint: " + wristAngleSetpoint + " and shoulderAngleSetpoint: " + shoulderAngleSetpoint);
        arm.setWristAnglePosition(wristAngleSetpoint);
        arm.setShoulderAnglePosition(shoulderAngleSetpoint);
        this.arm.setCurrentPose(pose);
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
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
