package frc.robot.commands;

//import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveBase;


public class TurnDistanceCommandButFastCommand extends CommandBase {
    float degrees;
    double p;
    double d;
    double maxSpeed;
    public float meters;
    double errorValueOfLastTick;

    public TurnDistanceCommandButFastCommand(int meters,double p,double d,double maxSpeed)  {
//actually move distance.

        this.p = p;
        this.d=d;
        this.errorValueOfLastTick=p;
        this.maxSpeed = maxSpeed;
this.meters=meters;

        addRequirements();






    }

    @Override
    public void initialize() {
        /*SlotConfiguration kpid = new SlotConfiguration();
        kpid.kD=d;
        kpid.kP=p;
        DriveBase.getInstance().SetPidDistance(meters);
        DriveBase.getInstance().setSlotLeft(kpid);
        DriveBase.getInstance().setSlotRight(kpid);
*/
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
