package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.services.Oi;

public class Collector extends SubsystemBase {
    public TalonFX Motor;

    private DigitalInput cubeBreak1;
    private DigitalInput cubeBreak2;
    private boolean isIntaking = false;
    private Collector(){

        cubeBreak1 = new DigitalInput(Constants.Collector.CUBE_BREAK_1_PORT);
        cubeBreak2 = new DigitalInput(Constants.Collector.CUBE_BREAK_2_PORT);

        double KP = 0.03;
        this.Motor = new TalonFX(Constants.Collector.MOTOR_ID);
        Motor.config_kP(0, KP);
        Motor.setNeutralMode(NeutralMode.Brake);
    }
    public void intake(){

        isIntaking = true;

        System.out.println("Intake is running");
          double objectDistancePerSec = Constants.Collector.INTAKE_SPEED;
          double objectDistancePerMs = objectDistancePerSec / 1000;
          double wheelRevsPerMs = objectDistancePerMs / Constants.Collector.WHEEL_CIRCUMFERENCE;
          double motorRevsPerMs = wheelRevsPerMs * Constants.Collector.GEAR_RATIO;
          double ticksPerMs = motorRevsPerMs * Constants.Falcon500.TICKS_PER_REV;
          System.out.println("the speed is" + ticksPerMs);

        // takes ticks per 100ms
        this.Motor.set(TalonFXControlMode.Velocity, ticksPerMs);
        System.out.println("Velocity set");
        //Motor.set(TalonFXControlMode.PercentOutput, Oi.Instance.getLeftY());
    }
    public void expel(){
        double objectDistancePerSec = Constants.Collector.EXPEL_SPEED;
        double objectDistancePerMs = objectDistancePerSec / 1000;
        double wheelRevsPerMs = objectDistancePerMs / Constants.Collector.WHEEL_CIRCUMFERENCE;
        double motorRevsPerMs = wheelRevsPerMs / Constants.Collector.GEAR_RATIO;
        double ticksPerMs = motorRevsPerMs * Constants.Falcon500.TICKS_PER_REV;
        // takes ticks per 100ms

        this.Motor.set(TalonFXControlMode.Velocity, ticksPerMs * -100);
    }

    //@Override
    //public void periodic() {

        //if(isIntaking && cubeCollected()) {
            //Motor.set(TalonFXControlMode.Velocity, 0);
            //isIntaking = false;
        //}


    //}
    private boolean cubeCollected() {
        if(cubeBreak1.get() == true || cubeBreak2.get() == true) {
            return true;
        }
        else {
            return false;
        }
    }
    public static Collector getInstance(){
        if ( instance == null){
            instance = new Collector();
        }
        return instance;
    }
    private static Collector instance;
    public void Setspeed(double speed) {
        this.Motor.set(TalonFXControlMode.Velocity, speed);
    }

}
