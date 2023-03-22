package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;


public class DriveToConeCommand extends CommandBase
{
    private final Drivebase drivebase;
    private double PixelError = 0; //can be anywhere from -160 to 160

    public DriveToConeCommand()
    {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        drivebase = Drivebase.GetDrivebase();
        addRequirements(drivebase);
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        double leftSpeed;
        double rightSpeed;
        if(Math.abs(PixelError) == PixelError)
        {
            leftSpeed = Constants.Drivebase.BASE_DRIVE_TO_CONE_SPEED + (Math.abs(PixelError) * Constants.Drivebase.DRIVE_TO_CONE_KP); //up
            rightSpeed = Constants.Drivebase.BASE_DRIVE_TO_CONE_SPEED - (Math.abs(PixelError) * Constants.Drivebase.DRIVE_TO_CONE_KP); //down
        }
        else
        {
            leftSpeed = Constants.Drivebase.BASE_DRIVE_TO_CONE_SPEED - (Math.abs(PixelError) * Constants.Drivebase.DRIVE_TO_CONE_KP); //down
            rightSpeed = Constants.Drivebase.BASE_DRIVE_TO_CONE_SPEED + (Math.abs(PixelError) * Constants.Drivebase.DRIVE_TO_CONE_KP); //up
        }


        leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
        rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);

        drivebase.runMotor(leftSpeed, rightSpeed);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {

    }
}
