package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Arm extends SubsystemBase {
    public TalonFX Motor = new TalonFX(5);
    public double encoderToAngleFactor = ((1.0 / Constants.Falcon500.TICKS_PER_REV) / Constants.Arm.GEAR_RATIO) * 360;
    public double lastAngle;
    public double setpoint;
    private final static Arm INSTANCE = new Arm();

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {
        Motor.selectProfileSlot(0, 0);
        Motor.config_kP(0, Constants.Arm.KP);
        updateKf(Constants.Arm.KF, Constants.Arm.RESTING_ANGLE);
        Motor.setNeutralMode(NeutralMode.Coast);
        //Motor.setNeutralMode(NeutralMode.Coast);
        Motor.configClosedloopRamp(0.1);
        Motor.configClosedLoopPeakOutput(0, 0.3);
        Motor.configNeutralDeadband(0.0);
        Motor.setInverted(InvertType.None);
        Motor.setSelectedSensorPosition(Constants.Arm.RESTING_ANGLE * encoderToAngleFactor);
    }

    public void setShoulderAnglePosition(double degrees) {
        double pos = degrees / encoderToAngleFactor;

        setpoint = degrees;
        updateKf(Constants.Arm.KF, setpoint);

        System.out.println("setting target: " + pos);
        Motor.set(TalonFXControlMode.Position, pos);
    }

    public double getShoulderAngle() {
        return Constants.Arm.RESTING_ANGLE + Motor.getSelectedSensorPosition() * encoderToAngleFactor;
    }

    public double getCurrentOutput() {
        return Motor.getMotorOutputPercent();
    }

    /*
    Fix motor scaling the kf based on the setpoint

    kf: the value you want to set the kf to

    results: divides the input kf by the setpoint and multiplies
    the result by 1024
    */
    private void configArmKF(double kf) {
        double pos = setpoint / encoderToAngleFactor;
        double kF = (kf / pos) * 1024;
        //lastAngle = pos;

        Motor.config_kF(0, kF);
    }

    /*
    Calculates a kf based on an input kf and an input position.
    newKF is the input kf multiplied by the sin of the input position.
    We do this because less kf is needed when the motor has less
    torque from the weight.

    kf: the input kf
    pos: the current position of the arm where 0 degrees is straight up

    results: calculates newKF and sets it using configArmKF

    */
    public void updateKf(double kf, double pos) {
        double newKF = kf * Math.sin(pos * Math.PI / 180f);

        System.out.println("Updating kf to " + newKF);

        //System.out.println("New kf: " + newKF);
        SmartDashboard.putNumber("newkf", newKF);

        configArmKF(newKF);
    }

    /*
    If the arm has moved more than 0.5 degrees since the kf was last
    updated then update it again.
    */
    @Override
    public void periodic() {
        double pos = getShoulderAngle();
        //System.out.println("Angle: " + pos);
        //SmartDashboard.putNumber("Angle:", pos);

        SmartDashboard.putNumber("setpoint", setpoint);

        if (Math.abs(pos - lastAngle) > Constants.Arm.ANGLE_UPDATE) {
            lastAngle = pos;
            System.out.println("updating kf");
            updateKf(Constants.Arm.KF, pos);
        }
    }
}