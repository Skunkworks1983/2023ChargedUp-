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
    private boolean weirdAngle;

    public SetArmPositionCommand(Arm.PoseType pose, double shoulderAngleSetpoint, double wristAngleSetpoint)
    {
        this.arm = Arm.getInstance();
        this.shoulderAngleSetpoint = shoulderAngleSetpoint;
        this.wristAngleSetpoint = wristAngleSetpoint;
        this.pose = pose;
        addRequirements(arm);
        weirdAngle = false;
    }

    @Override
    public void initialize()
    {
        if(Math.abs(arm.ShoulderMotor.getClosedLoopTarget() * Constants.Arm.SHOULDER_TICKS_TO_DEGREES - Constants.ArmPos.SCORE_CONE_WEIRD_SHOULDER) < 1)
        {
            weirdAngle = true;
            arm.setWristAnglePosition(wristAngleSetpoint);
        }
        else
        {
            arm.setWristAnglePosition(wristAngleSetpoint);
            arm.setShoulderAnglePosition(shoulderAngleSetpoint);
            weirdAngle = false;
        }
            System.out.println("set arm pos with wristAngleSetpoint: " + wristAngleSetpoint + " and shoulderAngleSetpoint: " + shoulderAngleSetpoint);
        arm.setCurrentPose(pose);
    }

    @Override
    public void execute()
    {
        if(weirdAngle && arm.getWristAngle() < Constants.Arm.WRIST_PARALLEL_WITH_SHOULDER)
        {
            arm.setShoulderAnglePosition(shoulderAngleSetpoint);
            weirdAngle = false;
        }
    }

    @Override
    public boolean isFinished()
    {
        return false;
        //return Math.abs(arm.ShoulderMotor.getClosedLoopError() * Constants.Arm.SHOULDER_TICKS_TO_DEGREES) < Constants.Arm.SHOULDER_TOLERANCE &&
        //Math.abs(arm.WristMotor.getClosedLoopError() * Constants.Arm.WRIST_TICKS_TO_DEGREES) < Constants.Arm.WRIST_TOLERANCE;
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
