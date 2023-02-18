package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class TurnDistanceWithProportionalControlCommand extends CommandBase {

    float degrees;
    double p;
    double maxSpeed;
    float startingDegrees;

    public TurnDistanceWithProportionalControlCommand(float degrees,double p,double maxSpeed) {//degrees should always be positive



        //if(degrees<0){degrees=-degrees;speed=-speed;}

        this.degrees = degrees;
        this.p = p;
        this.maxSpeed = maxSpeed;
        startingDegrees = (float) Drivebase.getInstance().getRotation();

        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double error = -degrees+(Drivebase.getInstance().getRotation()-startingDegrees);

        double newSpeed = error*p;
        if(newSpeed>0)newSpeed = Math.min(maxSpeed,newSpeed);
        if(newSpeed<0)newSpeed = Math.max(-maxSpeed,newSpeed);

        Drivebase.getInstance().setLeft(newSpeed);
        Drivebase.getInstance().setRight(-newSpeed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        System.out.println(((Drivebase.getInstance().getRotation()-startingDegrees)-degrees));
        //if((Math.abs(DriveBase.getInstance().getRotation()-startingDegrees)-degrees)<4)return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Drivebase.getInstance().setLeft(0);
        Drivebase.getInstance().setRight(0);
    }
}
