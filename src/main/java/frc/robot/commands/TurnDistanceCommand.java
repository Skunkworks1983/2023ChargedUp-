package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;


public class TurnDistanceCommand extends CommandBase {

    float degrees;
    float speed;
    float startingDegrees;

    public TurnDistanceCommand(float degrees,float speed) {//degrees should always be positive
        if(degrees<0){degrees=-degrees;speed=-speed;}

        this.degrees = degrees;
        this.speed = speed;
        startingDegrees = (float) Drivebase.getInstance().getRotation();

        addRequirements();
    }

    @Override
    public void initialize() {
        Drivebase.getInstance().setLeft(speed);
        Drivebase.getInstance().setRight(-speed);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        if(Math.abs(Drivebase.getInstance().getRotation()-startingDegrees)>degrees)return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Drivebase.getInstance().setLeft(0);
        Drivebase.getInstance().setRight(0);
    }
}
