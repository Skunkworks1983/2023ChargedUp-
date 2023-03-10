package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Arm extends SubsystemBase {

    public TalonFX ShoulderMotor = new TalonFX(Constants.Arm.SHOULDER_MOTOR_ID);
    public TalonFX WristMotor = new TalonFX(Constants.Arm.WRIST_MOTOR_DEVICE_NUMBER);
    public double shoulderEncoderToAngleFactor = ((1.0 / Constants.Falcon500.TICKS_PER_REV) / Constants.Arm.SHOULDER_GEAR_RATIO) * 360;
    public double wristEncoderToAngleFactor = ((1.0 / Constants.Falcon500.TICKS_PER_REV) / Constants.Arm.WRIST_GEAR_RATIO) * 360;

    public double peakOutput;
    public double lastAngle;
    public double setpoint;
    private final static Arm INSTANCE = new Arm();

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {
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

        SmartDashboard.putNumber("should be", Constants.Arm.SHOULDER_RESTING_ANGLE / Constants.Arm.SHOULDER_TICKS_TO_DEGREES);
        //SmartDashboard.putNumber("constructor current", ShoulderMotor.getSelectedSensorPosition());

        updateKf(Constants.Arm.SHOULDER_KF, Constants.Arm.SHOULDER_RESTING_ANGLE, peakOutput);
    }

    public void setShoulderAnglePosition(double degrees) {
        double pos = degrees / Constants.Arm.SHOULDER_TICKS_TO_DEGREES;

        setpoint = pos;
        updateKf(Constants.Arm.SHOULDER_KF, degrees, peakOutput);

        System.out.println("setting target: " + pos);
        ShoulderMotor.set(TalonFXControlMode.Position, pos);
    }

    public void setWristAnglePosition(double degrees) {
        double pos = degrees / Constants.Arm.WRIST_TICKS_TO_DEGREES;

        setpoint = pos;
        System.out.println("setting target wrist: " + pos);
        WristMotor.set(TalonFXControlMode.Position, pos);
    }

    public double getShoulderAngle() {
        return ShoulderMotor.getSelectedSensorPosition() * Constants.Arm.SHOULDER_TICKS_TO_DEGREES;
    }

    public double getWristAngle() {
        return WristMotor.getSelectedSensorPosition() * Constants.Arm.WRIST_TICKS_TO_DEGREES;
    }

    public double getShoulderCurrentOutput() {
        return ShoulderMotor.getMotorOutputPercent();
    }

    public double getWristCurrentOutput() {
        return WristMotor.getMotorOutputPercent();
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
        config.closedLoopPeakOutput = peakOutput;

        ShoulderMotor.configureSlot(config);
    }

    /*
    Fix motor scaling the kf based on the setpoint

    kf: the value you want to set the kf to

    results: divides the input kf by the setpoint and multiplies
    the result by 1024
    */
    private void configArmKF(double kf, double peakOutput) {
        double pos = setpoint;
        double kF = (kf / pos) * 1023;
        //lastAngle = pos;

        configArmSlot(Constants.Arm.KP, Constants.Arm.KI, 0, kF, peakOutput);
    }

    public void SetWristSpeed(double speed) {

        WristMotor.set(TalonFXControlMode.PercentOutput, speed);
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
    public void updateKf(double kf, double pos, double peakOutput) {
        double newKF = kf * Math.sin(pos * Math.PI / 180f);

//        System.out.println("Updating kf to " + newKF);
//        System.out.println("pos: " + pos);

        //System.out.println("New kf: " + newKF);
        SmartDashboard.putNumber("kf", newKF * 1023);

        configArmKF(newKF, peakOutput);
    }

    public boolean limitSwitchOutput(int limitSwitchPort) {
        if (limitSwitchPort == Constants.Arm.SHOULDER_LIMIT_SWITCH_FRONT) {
            return ShoulderMotor.getSensorCollection().isFwdLimitSwitchClosed() == 1;
        } else if (limitSwitchPort == Constants.Arm.SHOULDER_LIMIT_SWITCH_BACK) {
            return ShoulderMotor.getSensorCollection().isRevLimitSwitchClosed() == 1;
        } else if (limitSwitchPort == Constants.Arm.WRIST_LIMIT_SWITCH) {
            return WristMotor.getSensorCollection().isRevLimitSwitchClosed() == 1;
        } else {
            System.out.println("Incorrect limit switch constant");
            return false;
        }
    }

    public void SetPercentOutput(double percent) {

        ShoulderMotor.set(ControlMode.PercentOutput, percent);

    }

    public void SetBrakeMode(boolean enable, TalonFX Motor) {
        if (enable) {

            Motor.setNeutralMode(NeutralMode.Brake);

        } else {

            Motor.setNeutralMode(NeutralMode.Coast);
        }
    }

    @Override
    public void periodic() {
        double wristPos = getWristAngle();
        double shoulderPos = getShoulderAngle();
        //System.out.println("Angle: " + wristPos);
        SmartDashboard.putNumber("Error:", WristMotor.getClosedLoopError());

        SmartDashboard.putNumber("setpoint", setpoint);
        SmartDashboard.putNumber("position", shoulderPos);

        if (Math.abs(shoulderPos - lastAngle) > Constants.Arm.SHOULDER_ANGLE_UPDATE) {
            lastAngle = shoulderPos;
        }
        //System.out.println("updating kf");


        if (getShoulderAngle() < Constants.Arm.SHOULDER_SETPOINT_1) {

            peakOutput = Constants.Arm.SETPOINT_1_PEAK;
            System.out.println(peakOutput);

        } else if (getShoulderAngle() >= Constants.Arm.SHOULDER_SETPOINT_1 &&
                getShoulderAngle() < Constants.Arm.SHOULDER_SETPOINT_2) {

            peakOutput = Constants.Arm.SETPOINT_2_PEAK;
            System.out.println(peakOutput);


        } else if (getShoulderAngle() >= Constants.Arm.SHOULDER_SETPOINT_2 &&
                getShoulderAngle() < Constants.Arm.SHOULDER_SETPOINT_3) {

            peakOutput = Constants.Arm.SETPOINT_3_PEAK;
            System.out.println(peakOutput);

        } else if (getShoulderAngle() >= Constants.Arm.SHOULDER_SETPOINT_3 &&
                getShoulderAngle() < Constants.Arm.SHOULDER_SETPOINT_4) {

            peakOutput = Constants.Arm.SETPOINT_4_PEAK;
            System.out.println(peakOutput);

        } else if (getShoulderAngle() >= Constants.Arm.SHOULDER_SETPOINT_4) {

            peakOutput = Constants.Arm.SETPOINT_5_PEAK;}
            System.out.println(peakOutput);

        SmartDashboard.putNumber("wrist position", wristPos);
            SmartDashboard.putNumber("shoulder position", shoulderPos);
            SmartDashboard.putNumber("Motor output: ", ShoulderMotor.getMotorOutputPercent());
            if (Math.abs(wristPos - lastAngle) > Constants.Arm.SHOULDER_ANGLE_UPDATE) {
                lastAngle = wristPos;
                //System.out.println("updating kf");
                updateKf(Constants.Arm.SHOULDER_KF, shoulderPos, peakOutput);
            }


        }

    }
