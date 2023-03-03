package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
            System.out.println("returns true");
            return true;

        }
        else {
            System.out.println("returns false");
            return false;
        }

    }
    public boolean isHoldingCone() {
        SmartDashboard.putNumber("Colletor current", GetCollectorCurrent());
        if(GetCollectorCurrent() >= Constants.Collector.CONE_COLLECT_AMP_THRESHOLD) {
            return true;
        }
        else{
            return false;
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
