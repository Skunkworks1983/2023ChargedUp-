package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;


public class DriveDistanceCommand extends CommandBase
{
    private final Drivebase drivebase;
    private final double distanceFT;
    private double startDistanceFT;
    private double finishDistanceFT;
    private int direction;
    private double startDegree;

    /**
     *
     * @param drivebase what drivebase to use
     * @param distanceFT The direction and distance in which to go
     */
    public DriveDistanceCommand(Drivebase drivebase, double distanceFT)
    {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.drivebase = drivebase;
        this.distanceFT = distanceFT;
        this.startDegree = startDegree;
        addRequirements(drivebase);
    }

    @Override
    public void initialize()
    {

        startDegree = drivebase.getHeading();
        startDistanceFT = drivebase.getPosLeft();
        finishDistanceFT = startDistanceFT+distanceFT;
        startDegree = drivebase.getHeading();
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
    }

    @Override
    public void execute()
    {
       // startDegree - ;
        double error = finishDistanceFT - drivebase.getPosLeft();
        if(error < 0)
        {
            direction = -1;
        }
        else
        {
            direction = 1;
        }
        double speed = Constants.Drivebase.DISTANCE_KP * error + Constants.Drivebase.DRIVEBASE_KF * direction;
        if (speed > 0.3)
        {
            speed = 0.3;
        }
        else if (speed < -0.3)
        {
            speed = -0.3;
        }
       // double speedLeft = speed + Math.max(Math.min(Constants.Drivebase.ANGLE_KP*(startDegree - drivebase.getHeading()), 0.25), -0.25);
       // double speedRight = speed - Math.max(Math.min(Constants.Drivebase.ANGLE_KP*(startDegree - drivebase.getHeading()), 0.25), -0.25);
        drivebase.runMotor(speed, speed);
        //SmartDashboard.putNumber("FT moved", drivebase.getPosLeft()-startDistanceFT);
    }

    @Override
    public boolean isFinished()
    {
        return Math.abs(drivebase.getPosLeft() - finishDistanceFT) < 0.1;
    }

    @Override
    public void end(boolean interrupted)
    {
        drivebase.runMotor(0, 0);
        System.out.println("Ended at: "+drivebase.getPosLeft());
    }
}
