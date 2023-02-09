package frc.robot.commands;
//import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Limelight;

public class RunTankDriveCommand extends CommandBase {

    public RunTankDriveCommand() {

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        DriveBase.getInstance().setLeft(OI.getInstance().getLeftJoystick());
        DriveBase.getInstance().setRight(OI.getInstance().getRightJoystick());
        //System.out.println(DriveBase.getInstance().getRotation());
        //Limelight.UpdateTable();
        //System.out.println("("+Limelight.GetInstance().poseData[0]+","+Limelight.GetInstance().poseData[1]+","+Limelight.GetInstance().poseData[2]+")"+":"+Limelight.GetInstance().currentTag);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        DriveBase.getInstance().setLeft(0);
        DriveBase.getInstance().setRight(0);
    }
}
