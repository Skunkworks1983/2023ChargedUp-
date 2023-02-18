package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;


public class DriveOntoChargeStationCommand extends CommandBase {
float speed;
    public DriveOntoChargeStationCommand(float speed) {
        this.speed=speed;
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Drivebase.getInstance().setLeft(-speed);
        Drivebase.getInstance().setRight(-speed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        System.out.println("HELLOW WORLD" + Math.abs(Drivebase.getInstance().getPitch()));
        if(Math.abs(Drivebase.getInstance().getPitch())>10){ return true;}
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        Drivebase.getInstance().setLeft(0);
        Drivebase.getInstance().setRight(0);
    }
}
