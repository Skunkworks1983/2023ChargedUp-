package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;


public class TurnDistanceCommand extends CommandBase {

    float degrees;
    float speed;
    float startingDegrees;

    public TurnDistanceCommand(float degrees,float speed) {//degrees should always be positive
        if(degrees<0){degrees=-degrees;speed=-speed;}

        this.degrees = degrees;
        this.speed = speed;
        startingDegrees = (float)DriveBase.getInstance().getRotation();

        addRequirements();
    }

    @Override
    public void initialize() {
        DriveBase.getInstance().setLeft(speed);
        DriveBase.getInstance().setRight(-speed);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        if(Math.abs(DriveBase.getInstance().getRotation()-startingDegrees)>degrees)return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        DriveBase.getInstance().setLeft(0);
        DriveBase.getInstance().setRight(0);
    }
}
