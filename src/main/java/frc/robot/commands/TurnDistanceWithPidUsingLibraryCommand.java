package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;


public class TurnDistanceWithPidUsingLibraryCommand extends CommandBase {

    float degrees;
    double p;
    double maxSpeed;
    float startingDegrees;
    double errorValueOfLastTick;
    public double d;
    public PIDController pid;

    public TurnDistanceWithPidUsingLibraryCommand(float degrees,double p,double d,double maxSpeed) {//degrees should always be positive
        //if(degrees<0){degrees=-degrees;speed=-speed;
        pid = new PIDController(p,0,d);
        this.degrees = degrees;
        this.p = p;
        this.d=d;
        this.errorValueOfLastTick=p;
        this.maxSpeed = maxSpeed;

        addRequirements();
    }

    @Override
    public void initialize() {
        pid.setSetpoint(DriveBase.getInstance().getRotation()+degrees);
        startingDegrees = (float)DriveBase.getInstance().getRotation();
    }

    @Override
    public void execute() {
        double error = -degrees+(DriveBase.getInstance().getRotation()-startingDegrees);
        //System.out.println(error);
        double newSpeed = pid.calculate(DriveBase.getInstance().getRotation());
    System.out.println("("+DriveBase.getInstance().getRotation()+","+pid.getSetpoint()+")");
        //double slope = (error-errorValueOfLastTick)*(1/50);
        //newSpeed+=slope*d;

        if(newSpeed>0)newSpeed = Math.min(maxSpeed,newSpeed);
        if(newSpeed<0)newSpeed = Math.max(-maxSpeed,newSpeed);

        DriveBase.getInstance().setLeft(-newSpeed);
        DriveBase.getInstance().setRight(newSpeed);

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        //System.out.println(((DriveBase.getInstance().getRotation()-startingDegrees)-degrees));
        //if((Math.abs(DriveBase.getInstance().getRotation()-startingDegrees)-degrees)<4)return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        DriveBase.getInstance().setLeft(0);
        DriveBase.getInstance().setRight(0);
    }
}
