package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Collector extends SubsystemBase {

    public TalonFX Motor;

    private DigitalInput cubeBreak1;
    private DigitalInput cubeBreak2;

    private Collector(){

        cubeBreak1 = new DigitalInput(Constants.Collector.CUBE_BREAK_1_PORT);
        cubeBreak2 = new DigitalInput(Constants.Collector.CUBE_BREAK_2_PORT);

        this.Motor = new TalonFX(Constants.Collector.MOTOR_ID);
        Motor.config_kP(0, Constants.Collector.K_P);
        Motor.setNeutralMode(NeutralMode.Brake);
        Motor.setInverted(true);
    }
    public boolean isHoldingCube() {

        if(cubeBreak1.get() == false && cubeBreak2.get() == false) {
            return true;
        }
        else{
            return false;
        }

    }

    public boolean isIntaking () {

        if (cubeBreak1.get() == cubeBreak2.get()) {

            return false;
        } else {

            return true;
        }
    }

    public double GetCollectorCurrent()
    {
        return Motor.getSupplyCurrent();
    }

    public boolean cubeCollectedExpel() {

        if(cubeBreak1.get() == true || cubeBreak2.get() == true) {
            return false;

        }
        else {
            return true;
        }
    }
    public boolean isHoldingCone() {
        return Motor.getSupplyCurrent() >= Constants.Collector.CONE_COLLECT_AMP_THRESHOLD;
    }
    public boolean coneCurrentHolding() {
        return Motor.getSupplyCurrent() >= Constants.Collector.CONE_HOLDING_AMPS;
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
    public void SetPercentOutput(double speed) {
        this.Motor.set(TalonFXControlMode.PercentOutput, speed);
    }


}
