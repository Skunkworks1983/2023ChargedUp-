package frc.robot.commands.arm;

import com.fasterxml.jackson.databind.ser.impl.PropertySerializerMap;
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
        double shoulderError = arm.getShoulderAngle() - shoulderAngleSetpoint;
        double wristError = arm.getWristAngle() - wristAngleSetpoint;
        SmartDashboard.putNumber("shoulder error: ", arm.getShoulderAngle());
        SmartDashboard.putNumber("wrist error: ", arm.getWristAngle());

        return Math.abs(shoulderError) < Constants.Arm.SHOULDER_TOLERANCE &&
               Math.abs(wristError) < Constants.Arm.WRIST_TOLERANCE;
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
