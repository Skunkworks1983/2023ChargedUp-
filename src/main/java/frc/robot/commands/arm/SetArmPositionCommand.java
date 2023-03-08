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
    private boolean needWristSet = false;

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
        if(wristAngleSetpoint >= Constants.Arm.MAX_WRIST_ROTATION)
        {
            System.out.println("Not setting Wrist angle on initialize");
            needWristSet = true;
        }
        else
        {
            arm.setWristAnglePosition(wristAngleSetpoint);
        }
        arm.setShoulderAnglePosition(shoulderAngleSetpoint);
    }

    @Override
    public void execute()
    {
        if(needWristSet && arm.getShoulderAngle() >= Constants.Arm.SHOULDER_SAFE_WRIST_ANGLE)
        {
            arm.setWristAnglePosition(wristAngleSetpoint);
            needWristSet = false;
            System.out.println("Now Enabling Wrist");
        }
    }

    @Override
    public boolean isFinished()
    {
        //System.out.print("Shoulder error: " + Math.abs(arm.ShoulderMotor.getClosedLoopError() * Constants.Arm.SHOULDER_TICKS_TO_DEGREES));
        //System.out.println(" Wrist error: " + Math.abs(arm.WristMotor.getClosedLoopError() * Constants.Arm.WRIST_TICKS_TO_DEGREES));
        //return Math.abs(arm.ShoulderMotor.getClosedLoopError() * Constants.Arm.SHOULDER_TICKS_TO_DEGREES) < Constants.Arm.SHOULDER_TOLERANCE &&
        //       Math.abs(arm.WristMotor.getClosedLoopError() * Constants.Arm.WRIST_TICKS_TO_DEGREES) < Constants.Arm.WRIST_TOLERANCE;
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        System.out.println("Done!");
    }
}
