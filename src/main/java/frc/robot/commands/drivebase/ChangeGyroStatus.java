package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;


public class ChangeGyroStatus extends Command {

    private boolean gyroSet = false;
    private boolean status;

    public ChangeGyroStatus(boolean status) {
        this.status = status;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
        System.out.println("Setting gyro to " + status);

        Drivebase.GetDrivebase().setGyroStatus(status);

        gyroSet = true;
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return gyroSet;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
