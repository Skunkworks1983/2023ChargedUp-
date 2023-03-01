package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.multidrivebase.Drivebase;
import frc.robot.subsystems.multidrivebase.Drivebase4MotorTalonFX;


public class RotateWristByPowerCommand extends CommandBase {

    double speed;
    Arm arm = Arm.getInstance();

    public RotateWristByPowerCommand(double speed) {
        this.speed = speed;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
        System.out.println("command initialized at speed:" + speed);
    }

    @Override
    public void execute() {
        arm.SetWristSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ended");
        arm.SetWristSpeed(0);
    }
}
