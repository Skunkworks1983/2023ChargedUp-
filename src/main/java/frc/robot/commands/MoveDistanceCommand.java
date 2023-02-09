package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;


public class MoveDistanceCommand extends CommandBase {

    int ticksUntilFinishedLeft=0;
    int ticksUntilFinishedRight=0;
    int startingTicksLeft=0;
    int startingTicksRight=0;

    int endTicksLeft=0;
    int endTicksRight=0;
    double speed;

    public MoveDistanceCommand(int ticks,double speed){

        //System.out.println("move distance constructed to go " +ticks + " ticks!");

        this.speed = speed;
        ticksUntilFinishedLeft=ticks;
        ticksUntilFinishedRight=ticks;

        addRequirements();

    }

    @Override
    public void initialize() {


        startingTicksLeft = DriveBase.getInstance().getLeftTicks();
        startingTicksRight = DriveBase.getInstance().getRightTicks();

        System.out.println("initialized!");
        DriveBase.getInstance().setLeft(speed);
        DriveBase.getInstance().setRight(speed);
    }

    @Override
    public void execute() {


    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()

System.out.println((DriveBase.getInstance().getRightTicks() + "-" +startingTicksRight) + "," + ticksUntilFinishedRight);


        if(Math.abs(DriveBase.getInstance().getLeftTicks()-startingTicksLeft)>ticksUntilFinishedLeft) {
            return true;

        }
            if(Math.abs(DriveBase.getInstance().getRightTicks()-startingTicksRight)>ticksUntilFinishedRight){

                    return true;
            }


        //}

        //System.out.println("isFineshed returned false");
        return false;
    }

    @Override
    public void end(boolean interrupted) {

            DriveBase.getInstance().setLeft(0);
            DriveBase.getInstance().setRight(0);

    }
}

