package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;


public class RotateWithEncoderCommand extends CommandBase {

    private final Drivebase drivebase;
    private double targetDegree;

    public RotateWithEncoderCommand(Drivebase drivebase, double targetDegree) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.drivebase = drivebase;
        this.targetDegree = targetDegree;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
