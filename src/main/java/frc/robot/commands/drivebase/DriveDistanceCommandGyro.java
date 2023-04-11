package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;

import static java.lang.Double.NaN;


public class DriveDistanceCommandGyro extends CommandBase
{
    private final Drivebase drivebase;
    private final double distanceFT;
    private double startDistanceFT;
    private double finishDistanceFT;
    private int direction;
    private double startDegree;
    private double baseSpeed;
    private int periodicCounter;
    private PIDController pidController = new PIDController(Constants.Drivebase.ARCADE_DRIVE_KP, 0, Constants.Drivebase.ARCADE_DRIVE_KD);

    /**
     *
     * @param drivebase what drivebase to use
     * @param distanceFT The direction and distance in which to go
     */
    public DriveDistanceCommandGyro(Drivebase drivebase, double distanceFT, double baseSpeed)
    {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.baseSpeed = baseSpeed;
        this.drivebase = drivebase;
        this.distanceFT = distanceFT;
        this.startDegree = startDegree;
        addRequirements(drivebase);
    }

    @Override
    public void initialize()
    {

        startDistanceFT = drivebase.getPosLeft();
        finishDistanceFT = startDistanceFT+distanceFT;
        startDegree = drivebase.getHeading();
        if(startDegree == NaN)
        {
            startDegree = 0;
        }
        if(distanceFT > 0)
        {
            direction = 1;
        }
        else
        {
            direction = -1;
        }
        drivebase.SetBrakeMode(true);
        System.out.println("moving from " +startDistanceFT + " to " + finishDistanceFT);
        periodicCounter = 0;
    }

    @Override
    public void execute()
    {
        periodicCounter = (periodicCounter + 1) % 15;
        double headingError = startDegree - drivebase.getHeading();
        double error = finishDistanceFT - drivebase.getPosLeft();
        if(error < 0)
        {
            direction = -1;
        }
        else
        {
            direction = 1;
        }
        double speed = Constants.Drivebase.DISTANCE_KP * error + this.baseSpeed * direction;
        if (speed > Constants.Drivebase.MAX_DRIVE_DISTANCE_SPEED)
        {
            speed = Constants.Drivebase.MAX_DRIVE_DISTANCE_SPEED;
        }
        else if (speed < -Constants.Drivebase.MAX_DRIVE_DISTANCE_SPEED)
        {
            speed = -Constants.Drivebase.MAX_DRIVE_DISTANCE_SPEED;
        }
        double turnThrottle = pidController.calculate(drivebase.getHeading(), startDegree);
        double leftSpeed = speed + turnThrottle;
        double rightSpeed = speed - turnThrottle;
        drivebase.runMotor(leftSpeed, rightSpeed);
        if(periodicCounter == 0)
        {
            System.out.println("DriveDistance current pos: " + drivebase.getPosLeft());
        }
    }

    @Override
    public boolean isFinished()
    {
        return Math.abs(drivebase.getPosLeft() - finishDistanceFT) < 0.1;
    }

    @Override
    public void end(boolean interrupted)
    {
        Arm.getInstance().SetLightMode(Constants.Lights.CUBE);
        drivebase.runMotor(0, 0);
        System.out.println("Ended at: "+drivebase.getPosLeft());
    }
}
