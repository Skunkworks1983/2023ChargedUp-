package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Collector extends SubsystemBase {
    TalonFX Motor;
    private Collector(){
        this.Motor = new TalonFX(0);
    }
    public void intake(){
        double objectDistancePerSec = 4 / 0.5; // 4in in half a sec
        double objectDistancePerMs = objectDistancePerSec / 1000;
        double wheelRevsPerMs = objectDistancePerMs / Constants.Collector.WHEEL_CIRCUMFERENCE;
        double motorRevsPerMs = wheelRevsPerMs / Constants.Collector.GEAR_RATIO;
        double ticksPerMs = motorRevsPerMs * Constants.Falcon500.TICKS_PER_REV;

        // takes ticks per 100ms
        this.Motor.set(TalonFXControlMode.Velocity, ticksPerMs * 100);
    }
    public void expel(){

    }
    public static Collector getInstance(){
        if ( instance == null){
            instance = new Collector();
        }
        return instance;
    }
    private static Collector instance;
}
