package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;


public class DriveUntilChargeStationCommand extends CommandBase {
double speed;
double pitchToFinish;
    public DriveUntilChargeStationCommand(double speed,double pitchToFinish) {
        this.speed=speed;
        this.pitchToFinish=pitchToFinish;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        Drivebase.GetDrivebase().runMotor(speed, speed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        if(Math.abs(Drivebase.GetDrivebase().getPitch())>pitchToFinish)return true;
        return false;
    }


    @Override
    public void end(boolean interrupted)
    {

        Arm.getInstance().SetLightMode(Constants.Lights.CONE);
        Drivebase.GetDrivebase().runMotor(0, 0);
        System.out.println("Ending Drive Distance Command");
    }
}
