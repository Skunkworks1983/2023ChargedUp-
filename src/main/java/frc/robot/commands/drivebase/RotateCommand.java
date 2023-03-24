package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
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
   private PIDController pidController = new PIDController(Constants.Drivebase.ANGLE_KP, 0.0032, Constants.Drivebase.ANGLE_KD,Constants.Drivebase.DRIVEBASE_KF);

    public RotateCommand(Drivebase drivebase, double degree)
    {
        addRequirements(drivebase);
        this.drivebase = drivebase;
        this.degree = degree;
        pidController.setIntegratorRange(-0.2, 0.2);
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
        double startTime = Timer.getFPGATimestamp();
        System.out.print("err: " + (finishDegree-drivebase.getHeading()));
        double speed = 0;
        speed = pidController.calculate(drivebase.getHeading(), finishDegree);
        if(speed > 0.5)
        {
            speed =0.5;
        }
        else if(speed < -0.5)
        {
            speed = -0.5;
        }
        drivebase.runMotor(speed, -speed);
        System.out.println(" speed: " + speed);
        double endTime = Timer.getFPGATimestamp();
        System.out.println(endTime-startTime);
    }

    @Override
    public boolean isFinished()
    {
        if(Math.abs(drivebase.getHeading() - finishDegree)< Constants.Drivebase.THRESHOLD_ROTATE)
        {
            onTargetCount++;
        }
        else
        {
            onTargetCount = 0;
        }
        return onTargetCount >= 3;
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