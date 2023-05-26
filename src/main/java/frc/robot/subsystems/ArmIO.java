package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double shoulderSensorPos;
        public double wristSensorPos;
        public int shoulderRevSwitchClosed;
        public int shoulderFwdSwitchClosed;
        public int wristIsSwitchClosed;
        public double shoulderClosedLoopError;
        public double wristClosedLoopError;
        public double shoulderClosedLoopTarget;
    }

    public default void updateInputs(ArmIOInputsAutoLogged inputs) {
    }

    public default void setShoulderPos(double degrees) {
    }

    public default void setShoulderPercentOutput(double percent) {
    }

    public default void setWristPos(double degrees) {
    }

    public default void setWristSpeed(double speed) {
    }

    /*
    Configure the motor's kP, kI, kD, kF and closedLoopPeakOutput

    kP: the kP you want to set
    kI: the kI you want to set
    kD: the kD you want to set
    kF: the kF you want to set
    peakOutput: the closedLoopPeakOutput you want to set

    note: kP must be positive
    */
    public default void configArmSlot(double kP, double kI, double kD, double kF, double peakOutput) {
    }

    public default void setLightMode(int mode) {
    }

    public default void setWristBrakeMode(boolean enabled) {
    }

    public default void setShoulderBrakeMode(boolean enabled) {
    }

    public default boolean isArmReset() {
        return false;
    }

    public default void setWristSensorPosition(double sensorPos) {
    }

    public default void setShoulderSensorPosition(double sensorPos) {
    }

}
