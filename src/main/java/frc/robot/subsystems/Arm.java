package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Arm extends SubsystemBase {
    public TalonFX Motor = new TalonFX(Constants.Arm.MOTOR_ID);
    public double lastAngle;
    public double setpoint;
    private final static Arm INSTANCE = new Arm();

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {
        Motor.selectProfileSlot(0, 0);
        Motor.setNeutralMode(NeutralMode.Coast);
        Motor.configClosedloopRamp(0.1);
        Motor.configNeutralDeadband(0.0);
        Motor.setInverted(InvertType.None);
        Motor.setSelectedSensorPosition(Constants.Arm.RESTING_ANGLE / Constants.Arm.TICKS_TO_DEGREES);

        SmartDashboard.putNumber("should be", Constants.Arm.RESTING_ANGLE / Constants.Arm.TICKS_TO_DEGREES);
        SmartDashboard.putNumber("constructor current", Motor.getSelectedSensorPosition());

        updateKf(Constants.Arm.KF, Constants.Arm.RESTING_ANGLE);
    }

    public void setShoulderAnglePosition(double degrees) {
        double pos = degrees / Constants.Arm.TICKS_TO_DEGREES;

        setpoint = pos;
        updateKf(Constants.Arm.KF, degrees);

        System.out.println("setting target: " + pos);
        Motor.set(TalonFXControlMode.Position, pos);
    }

    public double getShoulderAngle() {
        return Motor.getSelectedSensorPosition() * Constants.Arm.TICKS_TO_DEGREES;
    }

    public double getCurrentOutput() {
        return Motor.getMotorOutputPercent();
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

    public void configArmSlot(double kP, double kI, double kD, double kF, double peakOutput) {
        SlotConfiguration config = new SlotConfiguration();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kF = kF;
        config.closedLoopPeakOutput = peakOutput;

        Motor.configureSlot(config);
    }

    /*
    Fix motor scaling the kf based on the setpoint

    kf: the value you want to set the kf to

    results: divides the input kf by the setpoint and multiplies
    the result by 1024
    */
    private void configArmKF(double kf) {
        double pos = setpoint;
        double kF = (kf / pos) * 1023;
        //lastAngle = pos;

        configArmSlot(Constants.Arm.KP, Constants.Arm.KI, 0, kF, Constants.Arm.PEAK_OUTPUT);
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

//        System.out.println("Updating kf to " + newKF);
//        System.out.println("pos: " + pos);

        //System.out.println("New kf: " + newKF);
        SmartDashboard.putNumber("kf", newKF * 1023);

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
        SmartDashboard.putNumber("position", pos);

        if (Math.abs(pos - lastAngle) > Constants.Arm.ANGLE_UPDATE) {
            lastAngle = pos;
            //System.out.println("updating kf");
            updateKf(Constants.Arm.KF, pos);
        }
    }
}