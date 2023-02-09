package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;


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
        DriveBase.getInstance().setLeft(-speed);
        DriveBase.getInstance().setRight(-speed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        System.out.println("HELLOW WORLD" + Math.abs(DriveBase.getInstance().getPitch()));
        if(Math.abs(DriveBase.getInstance().getPitch())>10){ return true;}
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        DriveBase.getInstance().setLeft(0);
        DriveBase.getInstance().setRight(0);
    }
}
