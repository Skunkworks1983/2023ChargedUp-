package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class RotateCommand extends CommandBase
{
    private final Drivebase drivebase;
    private double degree;
    private double startDegree;
    private double finishDegree;
    private int onTargetCount;

    public RotateCommand(Drivebase drivebase, double degree)
    {
        addRequirements();
        this.drivebase = drivebase;
        this.degree = degree;
    }

    @Override
    public void initialize()
    {
        startDegree = drivebase.getHeading();
        finishDegree = startDegree + degree;
        System.out.println("starting RotateCommand");
    }

    @Override
    public void execute()
    {
        double error = finishDegree - drivebase.getHeading();

        double speed = (Constants.Drivebase.ANGLE_KP * error) + Math.copySign(Constants.Drivebase.ROTATE_KF, error);
        if (speed > 0.5)
        {
            speed = 0.5;
        }
        if(speed < -0.5)
        {
            speed = -0.5;
        }
        drivebase.runMotor(speed, -speed);
    }

    @Override
    public boolean isFinished()
    {
        if(Math.abs(drivebase.getHeading() - finishDegree)< 1)
        {
            onTargetCount++;
        }
        else
        {
            onTargetCount = 0;
        }
        return onTargetCount >= Constants.Drivebase.THRESHOLD_ROTATE;
//        if(degree > 0)
//        {
//            return drivebase.getHeading() > finishDegree;
//        }
//        else
//        {
//            return drivebase.getHeading() < finishDegree;
//        }
    }

    @Override
    public void end(boolean interrupted)
    {
        drivebase.runMotor(0, 0);
        System.out.println("ending RotateCommand");
    }
}