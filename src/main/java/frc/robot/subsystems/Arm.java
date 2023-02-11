package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Arm extends SubsystemBase
{
    TalonFX Motor = new TalonFX(5);
    public double encoderToAngleFactor = ((1.0/ Constants.Falcon500.TICKS_PER_REV)/Constants.Arm.GEAR_RATIO)*360;

    private final static Arm INSTANCE = new Arm();

    public static Arm getInstance()

    {
        return INSTANCE;
    }

    private Arm()
    {
        Motor.config_kP(0, Constants.Arm.COLLECTOR_MOTOR_1_KP);
        Motor.selectProfileSlot(0, 0);
        Motor.setNeutralMode(NeutralMode.Brake);
        Motor.configClosedloopRamp(0.1);
        Motor.configClosedLoopPeakOutput(0, 0.55);
        Motor.configNeutralDeadband(0.0);
        Motor.setInverted(InvertType.None);
        Motor.setSelectedSensorPosition(0);
    }

    public void setShoulderAnglePosition(double encoderPosition)
    {
        System.out.println("setting target: " + encoderPosition / encoderToAngleFactor);
        Motor.set(TalonFXControlMode.Position, encoderPosition / encoderToAngleFactor);
    }

    public double getShoulderAngle()
    {
        return Motor.getSelectedSensorPosition() * encoderToAngleFactor;
    }
}

