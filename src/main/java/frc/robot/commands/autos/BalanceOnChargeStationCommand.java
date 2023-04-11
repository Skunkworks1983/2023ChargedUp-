package frc.robot.commands.autos;

//import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drivebase.DetectRangeSensorCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;


public class BalanceOnChargeStationCommand extends CommandBase {

    Double p;

    boolean hitDeadzone=false;
    double d;
    double maxSpeed;
    public float meters;
    double errorValueOfLastTick;
    double error=0;
    double integral=0;
    double lastError;
    double maxSpeedAfterDeadzone;

    double deadBand;
    double deadBandAfter;

    double i;
    public BalanceOnChargeStationCommand(double maxSpeedAfterDeadzone,double p, double d, double i, double maxSpeed,double deadBand,double deadBandAfter) {
        this.maxSpeedAfterDeadzone = maxSpeedAfterDeadzone;
        this.p = p;
        this.d=d;
        this.maxSpeed = maxSpeed;
        this.i=i;
        this.deadBand=deadBand;
        this.deadBandAfter=deadBandAfter;

        //defaults //p=.022//d=0//max=.085

        addRequirements(Drivebase.GetDrivebase());
    }


    @Override
    public void initialize() {
        System.out.println("BalanceOnChargeStation started");
    }

    @Override
    public void execute() {

        error= Drivebase.GetDrivebase().getPitch();
        double derivative = (error-lastError)*50;
        integral+=(error/50);
        double speed = (error*p)+(derivative*d)+ (integral*i);
        if(hitDeadzone) {
            if (speed > maxSpeedAfterDeadzone) {
                speed = maxSpeedAfterDeadzone;
            }
            if (speed < -maxSpeedAfterDeadzone) {
                speed = -maxSpeedAfterDeadzone;
            }
        }
        else{
            if (speed > maxSpeed) {
                speed = maxSpeed;
            }
            if (speed < -maxSpeed) {
                speed = -maxSpeed;
            }
        }
        if(speed>0&&Drivebase.GetDrivebase().getFrontRangeVoltage()> Constants.Drivebase.MAXIMUM_BALANCE_DISTANCE_FROM_GROUND_FRONT){speed=0;}
        if(speed<0&&Drivebase.GetDrivebase().getBackRangeVoltage()> Constants.Drivebase.MAXIMUM_BALANCE_DISTANCE_FROM_GROUND_BACK){speed=0;}
        if(hitDeadzone) {
            if (Math.abs(error) < deadBandAfter) {
                speed = 0;
                hitDeadzone = true;

                Arm.getInstance().SetLightMode(Constants.Lights.RED_WITH_WHITE);
            }
        }else{
            if (Math.abs(error) < deadBand) {
                speed = 0;
                hitDeadzone = true;
                Arm.getInstance().SetLightMode(Constants.Lights.BLUE_WITH_WHITE);
            }

        }
        Drivebase.GetDrivebase().runMotor(speed,speed);
        lastError=error;
    }
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("BalanceOnChargeStation ended. interupted:" + interrupted);
    }
}
