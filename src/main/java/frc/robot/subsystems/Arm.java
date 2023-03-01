package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Arm extends SubsystemBase {

    public TalonFX ShoulderMotor = new TalonFX(Constants.Arm.SHOULDER_MOTOR_ID);
    TalonFX wristMotor = new TalonFX(Constants.Arm.WRIST_MOTOR_DEVICE_NUMBER);
    public double encoderToAngleFactor = ((1.0 / Constants.Falcon500.TICKS_PER_REV) / Constants.Arm.SHOULDER_GEAR_RATIO) * 360;


    public double lastAngle;
    public double setpoint;
    private final static Arm INSTANCE = new Arm();
    private DigitalInput frontLimit = new DigitalInput(Constants.Arm.SHOULDER_LIMIT_SWITCH_FRONT);
    private DigitalInput backLimit = new DigitalInput(Constants.Arm.SHOULDER_LIMIT_SWITCH_BACK);

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {
        ShoulderMotor.selectProfileSlot(0, 0);
        ShoulderMotor.setNeutralMode(NeutralMode.Coast);
        ShoulderMotor.configClosedloopRamp(0.1);
        ShoulderMotor.configNeutralDeadband(0.0);
        ShoulderMotor.setInverted(InvertType.None);
        ShoulderMotor.setSelectedSensorPosition(Constants.Arm.SHOULDER_RESTING_ANGLE / Constants.Arm.SHOULDER_TICKS_TO_DEGREES);

        SmartDashboard.putNumber("should be", Constants.Arm.SHOULDER_RESTING_ANGLE / Constants.Arm.SHOULDER_TICKS_TO_DEGREES);
        SmartDashboard.putNumber("constructor current", ShoulderMotor.getSelectedSensorPosition());

        updateKf(Constants.Arm.SHOULDER_KF, Constants.Arm.SHOULDER_RESTING_ANGLE);
        wristMotor.setNeutralMode(NeutralMode.Brake);

        wristMotor.setNeutralMode(NeutralMode.Coast); //DEBUG testing
    }

    public void setShoulderAnglePosition(double degrees) {
        double pos = degrees / Constants.Arm.SHOULDER_TICKS_TO_DEGREES;

        setpoint = pos;
        updateKf(Constants.Arm.SHOULDER_KF, degrees);

        System.out.println("setting target: " + pos);
        //Motor.set(TalonFXControlMode.Position, pos); /todo
    }

    public double getShoulderAngle() {
        return ShoulderMotor.getSelectedSensorPosition() * Constants.Arm.SHOULDER_TICKS_TO_DEGREES;
    }

    public double geWristAngle() {
        return wristMotor.getSelectedSensorPosition() * Constants.Arm.WRIST_TICKS_TO_DEGREES;
    }

    public double getCurrentOutput() {
        return ShoulderMotor.getMotorOutputPercent();
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

        // TODO: change back
        config.closedLoopPeakOutput = 0;

        ShoulderMotor.configureSlot(config);
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

        configArmSlot(Constants.Arm.SHOULDER_KP, Constants.Arm.SHOULDER_KI, 0, kF, Constants.Arm.SHOULDER_PEAK_OUTPUT);
    }

    public void SetWristSpeed(double speed) {

        wristMotor.set(TalonFXControlMode.PercentOutput, speed);
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

    public boolean limitSwitchOutput(int limitSwitchPort)
    {
        if(limitSwitchPort == Constants.Arm.SHOULDER_LIMIT_SWITCH_FRONT)
        {
            return !frontLimit.get();
        }
        else if(limitSwitchPort == Constants.Arm.SHOULDER_LIMIT_SWITCH_BACK)
        {
            return !backLimit.get();
        }
        else
        {
            System.out.println("Incorrect limit switch constant");
            return false;
        }
    }

    public void SetPercentOutput(double percent) {

        ShoulderMotor.set(ControlMode.PercentOutput, percent);

    }
    public void SetBrakeMode(boolean enable)
    {
        if (enable) {

            ShoulderMotor.setNeutralMode(NeutralMode.Brake);

        } else {

            ShoulderMotor.setNeutralMode(NeutralMode.Coast);
        }
    }

    @Override
    public void periodic() {
        double pos = getShoulderAngle();
        //System.out.println("Angle: " + pos);
        //SmartDashboard.putNumber("Angle:", pos);

        SmartDashboard.putNumber("setpoint", setpoint);
        SmartDashboard.putNumber("position", pos);

        if (Math.abs(pos - lastAngle) > Constants.Arm.SHOULDER_ANGLE_UPDATE) {
            lastAngle = pos;
            //System.out.println("updating kf");
            updateKf(Constants.Arm.SHOULDER_KF, pos);
        }

    }
}
