package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;


public class TestVolocityModeCommand extends Command {

    int time=0;

    double acc=.004;


    int switchTime = 100;

    public TestVolocityModeCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        time++;
        double f=time;
        if(time>switchTime)f=(switchTime*2)-time;
        f*=acc;
        //
        //f is in mps
        f=Drivebase.GetDrivebase().metersToTicks(f);
        SmartDashboard.putNumber("speed",f);
        System.out.println(f);
        Drivebase.GetDrivebase().setLeftMeters(f);
        Drivebase.GetDrivebase().setRightMeters(f);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        if(time==2*switchTime)return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        Drivebase.GetDrivebase().setLeftMeters(0);
        Drivebase.GetDrivebase().setRightMeters(0);
    }
}
