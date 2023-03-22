package frc.robot.commands.autos;

//import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drivebase.DetectRangeSensorCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
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
    boolean isRedAlliance;
    Arm arm;

    double i;
    public BalanceOnChargeStationCommand(double p, double d, double i, double maxSpeed) {

        this.p = p;
        this.d=d;
        this.maxSpeed = maxSpeed;
        this.i=i;
        this.arm = Arm.getInstance();
        //defaults //p=.022//d=0//max=.085

        addRequirements(Drivebase.GetDrivebase());
    }


    @Override
    public void initialize()
    {
        isRedAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Red;
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
        if(Math.abs(error) < 5)
        {
            f = 0;
            if(isRedAlliance)
            {
                arm.SetLightMode(Constants.Lights.RED_WITH_WHITE);
            }
            else
            {
                arm.SetLightMode(Constants.Lights.BLUE_WITH_WHITE);
            }
        }
        else
        {
            arm.SetLightMode(Constants.Lights.CENTER);
        }
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
        arm.SetLightMode(Constants.Lights.BLANK);
    }
}
