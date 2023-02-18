package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Collector extends SubsystemBase {
    private TalonFX Motor;
    private DigitalInput cubeBreak1;
    private DigitalInput cubeBreak2;
    private boolean isIntaking = false;
    private Collector(){
        cubeBreak1 = new DigitalInput(Constants.Collector.CUBE_BREAK_1_PORT);
        cubeBreak2 = new DigitalInput(Constants.Collector.CUBE_BREAK_2_PORT);

        this.Motor = new TalonFX(Constants.Collector.MOTOR_ID);
    }
    public void intake(){
        isIntaking = true;

        double objectDistancePerSec = Constants.Collector.INTAKE_SPEED;
        double objectDistancePerMs = objectDistancePerSec / 1000;
        double wheelRevsPerMs = objectDistancePerMs / Constants.Collector.WHEEL_CIRCUMFERENCE;
        double motorRevsPerMs = wheelRevsPerMs / Constants.Collector.GEAR_RATIO;
        double ticksPerMs = motorRevsPerMs * Constants.Falcon500.TICKS_PER_REV;

        // takes ticks per 100ms
        this.Motor.set(TalonFXControlMode.Velocity, ticksPerMs * 100);
    }
    public void expel(){
        double objectDistancePerSec = Constants.Collector.EXPEL_SPEED;
        double objectDistancePerMs = objectDistancePerSec / 1000;
        double wheelRevsPerMs = objectDistancePerMs / Constants.Collector.WHEEL_CIRCUMFERENCE;
        double motorRevsPerMs = wheelRevsPerMs / Constants.Collector.GEAR_RATIO;
        double ticksPerMs = motorRevsPerMs * Constants.Falcon500.TICKS_PER_REV;
        // takes ticks per 100ms
        this.Motor.setSelectedSensorPosition(0);
        this.Motor.set(TalonFXControlMode.Velocity, ticksPerMs * 100);
    }

    @Override
    public void periodic() {
        if(isIntaking && cubeCollected()) {
            Motor.set(TalonFXControlMode.Velocity, 0);
            isIntaking = false;
        }
        if(Motor.getSelectedSensorPosition() >= Constants.Collector.EXPEL_DISTANCE_TICKS) {
            Motor.set(TalonFXControlMode.Velocity, 0);
        }

    }
    private boolean cubeCollected() {
        if(cubeBreak1.get() == true && cubeBreak2.get() == true) {
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
}
