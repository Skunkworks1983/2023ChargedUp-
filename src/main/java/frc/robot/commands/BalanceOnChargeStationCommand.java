package frc.robot.commands;

//import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;


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

    public BalanceOnChargeStationCommand(double p,double d,double i,double maxSpeed) {

        this.p = p;
        this.d=d;
        this.maxSpeed = maxSpeed;
this.i=i;
            //defaults //p=.022//d=0//max=.085

        addRequirements();
    }


    @Override
    public void initialize() {



    }

    @Override
    public void execute() {


        error=DriveBase.getInstance().getPitch();
        double derivative = (error-lastError)*50;
        integral+=(error/50);
        double f =(error*p)+(derivative*d)+ (integral*i);
        if(f>maxSpeed)f=maxSpeed;
        if(f<-maxSpeed)f=-maxSpeed;

        DriveBase.getInstance().setLeft(f);
        DriveBase.getInstance().setRight(f);
        lastError=error;
        //System.out.println("BALANCING NOW: "+f);
    }
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
