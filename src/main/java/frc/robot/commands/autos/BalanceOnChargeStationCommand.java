package frc.robot.commands.autos;

//import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drivebase.DetectRangeSensorCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;


public class BalanceOnChargeStationCommand extends CommandBase {

    Double p;
    double d;
    double maxSpeed;
    public float meters;
    double errorValueOfLastTick;
    double error=0;
    double integral=0;
    double lastError;

    double i;
    public BalanceOnChargeStationCommand(double p, double d, double i, double maxSpeed) {

        this.p = p;
        this.d=d;
        this.maxSpeed = maxSpeed;
        this.i=i;

        //defaults //p=.022//d=0//max=.085

        addRequirements();
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
        double f =(error*p)+(derivative*d)+ (integral*i);
        if(f>maxSpeed)f=maxSpeed;
        if(f<-maxSpeed)f=-maxSpeed;
        if(f>0&&Drivebase.GetDrivebase().getFrontRangeVoltage()> Constants.Drivebase.MAXIMUM_BALANCE_DISTANCE_FROM_GROUND_FRONT){f=0;}
        if(f<0&&Drivebase.GetDrivebase().getBackRangeVoltage()> Constants.Drivebase.MAXIMUM_BALANCE_DISTANCE_FROM_GROUND_BACK){f=0;}

        Drivebase.GetDrivebase().runMotor(f,f);
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
