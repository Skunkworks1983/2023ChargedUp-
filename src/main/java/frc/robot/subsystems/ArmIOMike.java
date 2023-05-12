package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class ArmIOMike implements ArmIO {
    TalonFX ShoulderMotor = new TalonFX(Constants.Arm.SHOULDER_MOTOR_ID);
    TalonFX WristMotor = new TalonFX(Constants.Arm.WRIST_MOTOR_DEVICE_NUMBER);

    DigitalOutput lightBit0 = new DigitalOutput(Constants.Lights.LIGHT_BIT_0);
    DigitalOutput lightBit1 = new DigitalOutput(Constants.Lights.LIGHT_BIT_1);
    DigitalOutput lightBit2 = new DigitalOutput(Constants.Lights.LIGHT_BIT_2);
    DigitalOutput lightBit3 = new DigitalOutput(Constants.Lights.LIGHT_BIT_3);

    public ArmIOMike() {
        //Shoulder config
        ShoulderMotor.selectProfileSlot(0, 0);
        ShoulderMotor.setNeutralMode(NeutralMode.Brake);
        ShoulderMotor.configClosedloopRamp(0.1);
        ShoulderMotor.configNeutralDeadband(0.0);
        ShoulderMotor.setInverted(InvertType.None);
        ShoulderMotor.setSelectedSensorPosition(Constants.Arm.SHOULDER_RESTING_ANGLE / Constants.Arm.SHOULDER_TICKS_TO_DEGREES);

        //Wrist config
        WristMotor.selectProfileSlot(0, 0);
        WristMotor.setNeutralMode(NeutralMode.Brake);
        WristMotor.configClosedloopRamp(0.1);
        WristMotor.configNeutralDeadband(0.0);
        WristMotor.setInverted(InvertType.None);
        WristMotor.setSelectedSensorPosition(Constants.Arm.WRIST_RESTING_ANGLE / Constants.Arm.WRIST_TICKS_TO_DEGREES);
        WristMotor.configClosedLoopPeakOutput(0, Constants.Arm.WRIST_PEAK_OUTPUT);
        WristMotor.config_kP(0, Constants.Arm.WRIST_KP);
        WristMotor.config_kI(0, Constants.Arm.WRIST_KI);
        WristMotor.config_kD(0, Constants.Arm.WRIST_KD);
        WristMotor.config_kF(0, Constants.Arm.WRIST_KF);
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs.shoulderSensorPos = ShoulderMotor.getSelectedSensorPosition();
        inputs.wristSensorPos = WristMotor.getSelectedSensorPosition();

        TalonFXSensorCollection shoulderSensorCollection = ShoulderMotor.getSensorCollection();
        inputs.shoulderFwdSwitchClosed = shoulderSensorCollection.isFwdLimitSwitchClosed();
        inputs.shoulderRevSwitchClosed = shoulderSensorCollection.isRevLimitSwitchClosed();

        inputs.wristIsSwitchClosed = WristMotor.getSensorCollection().isRevLimitSwitchClosed();
    }

    public void setShoulderPos(double pos) {
        ShoulderMotor.set(TalonFXControlMode.Position, pos);
    }

    public void setShoulderPercentOutput(double percent) {
        ShoulderMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setWristPos(double pos) {
        WristMotor.set(TalonFXControlMode.Position, pos);
    }

    public void setWristSpeed(double speed) {
        WristMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void configArmSlot(double kP, double kI, double kD, double kF, double peakOutput) {
        SlotConfiguration config = new SlotConfiguration();

        if (DriverStation.isAutonomous()) {
            config.kP = Constants.Arm.SHOULDER_KP_AUTO;
            config.closedLoopPeakOutput = Constants.Arm.SHOULDER_PEAK_OUTPUT_AUTO;
        } else {
            config.kP = kP;
            config.closedLoopPeakOutput = peakOutput;
        }
        config.kI = kI;
        config.kD = kD;
        config.kF = kF;

        // TODO: change back

        ShoulderMotor.configureSlot(config);
    }

    public void setLightMode(int mode) {
        lightBit0.set((mode & 0x01) == 0x01);
        lightBit1.set((mode & 0x02) == 0x02);
        lightBit2.set((mode & 0x04) == 0x04);
        lightBit3.set((mode & 0x08) == 0x08);
    }


    // TODO: finish below functions
    public void setWristBrakeMode(boolean enabled) {
    }

    public void setShoulderBrakeMode(boolean enabled) {
    }

    public boolean isArmReset() {
        return false;
    }
}
