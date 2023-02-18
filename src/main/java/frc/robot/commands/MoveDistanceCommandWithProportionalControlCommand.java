package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;


public class MoveDistanceCommandWithProportionalControlCommand extends CommandBase {

    int ticksUntilFinishedLeft=0;
    int ticksUntilFinishedRight=0;
    int startingTicksLeft=0;
    int startingTicksRight=0;


    int endTicksLeft=0;
    int endTicksRight=0;
    double maxSpeed;
    double p;

    public MoveDistanceCommandWithProportionalControlCommand(int ticks,double p,double maxSpeed){

        //System.out.println("move distance constructed to go " +ticks + " ticks!");
        this.maxSpeed = maxSpeed;
        this.p = p;
        ticksUntilFinishedLeft=ticks;
        ticksUntilFinishedRight=ticks;

        addRequirements();

    }

    @Override
    public void initialize() {


        startingTicksLeft = Drivebase.getInstance().getLeftTicks();
        startingTicksRight = Drivebase.getInstance().getRightTicks();

        //DriveBase.getInstance().setLeft(speed);
        //DriveBase.getInstance().setRight(speed);
    }

    @Override
    public void execute() {
//                  0                   20                         25                                5
        double leftError = ticksUntilFinishedLeft - (Drivebase.getInstance().getLeftTicks()-startingTicksLeft);//DriveBase.getInstance().getLeftTicks()-startingTicksLeft;
        double rightError = ticksUntilFinishedRight - (Drivebase.getInstance().getRightTicks()-startingTicksRight);
        double newSpeed =(leftError+rightError)*p/2;
        if(newSpeed>0)newSpeed = Math.min(maxSpeed,newSpeed);
        if(newSpeed<0)newSpeed = Math.max(-maxSpeed,newSpeed);
        Drivebase.getInstance().setLeft(newSpeed);
        Drivebase.getInstance().setRight(newSpeed);
        System.out.println("movedistance speed: " +newSpeed);

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()

        System.out.println((Drivebase.getInstance().getRightTicks() + "-" +startingTicksRight) + "," + ticksUntilFinishedRight);


        return(Math.abs(Drivebase.getInstance().getLeftTicks()-startingTicksLeft)>ticksUntilFinishedLeft);

        //return (Math.abs(DriveBase.getInstance().getRightTicks()-startingTicksRight)>ticksUntilFinishedRight);


        //}

        //System.out.println("isFineshed returned false");
        //return false;
    }

    @Override
    public void end(boolean interrupted) {

        Drivebase.getInstance().setLeft(0);
        Drivebase.getInstance().setRight(0);

    }
}

