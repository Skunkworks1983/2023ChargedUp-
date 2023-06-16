package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.services.Oi;
import frc.robot.subsystems.Arm;

public class SetArmPositionCommand extends CommandBase
{

    private final Arm arm;
    private final Oi oi;
    private double shoulderAngleSetpoint;
    private double wristAngleSetpoint;
    private Constants.ArmPose armPose;
    private boolean weirdAngle;

    public SetArmPositionCommand(Constants.ArmPose armPose)
    {
        this.armPose = armPose;

        oi = Oi.Instance;
        this.arm = Arm.getInstance();


        addRequirements(arm);
        weirdAngle = false;
    }

    @Override
    public void initialize()
    {
        arm.setCurrentPosition(armPose);
        Constants.ArmPose pos = arm.getCurrentPose();

        this.shoulderAngleSetpoint = pos.shoulderAngle;
        this.wristAngleSetpoint = pos.wristAngle;
        double target = arm.ShoulderMotor.getClosedLoopTarget();
        if(Math.abs(target * Constants.Arm.SHOULDER_TICKS_TO_DEGREES - Constants.ArmPose.FLOOR_WEIRD.shoulderAngle) < 1 ||
                Math.abs(target * Constants.Arm.SHOULDER_TICKS_TO_DEGREES - Constants.ArmPos.SINGLE_SUBSTATION_CONE) < 1)
        {
            weirdAngle = true;
            arm.setWristAnglePosition(wristAngleSetpoint);
            System.out.println("Weird angle is true");
        }
        else
        {
            arm.setWristAnglePosition(wristAngleSetpoint);
            arm.setShoulderAnglePosition(shoulderAngleSetpoint);
            weirdAngle = false;
        }
            System.out.println("set arm pos with wristAngleSetpoint: " + wristAngleSetpoint + " and shoulderAngleSetpoint: " + shoulderAngleSetpoint);
    }

    @Override
    public void execute()
    {
        if(weirdAngle && arm.getWristAngle() < Constants.Arm.WRIST_PARALLEL_WITH_SHOULDER)
        {
            arm.setShoulderAnglePosition(shoulderAngleSetpoint);
            weirdAngle = false;
            System.out.println("weird/single substation angle is no longer true, arm running");
        }
        System.out.println("error: " + arm.ShoulderMotor.getClosedLoopError() * Constants.Arm.SHOULDER_TICKS_TO_DEGREES);
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
