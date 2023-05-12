package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class Arm extends SubsystemBase {
    private Constants.ArmPose currentPose;

    public double shoulderEncoderToAngleFactor = ((1.0 / Constants.Falcon500.TICKS_PER_REV) / Constants.Arm.SHOULDER_GEAR_RATIO) * 360;
    public double wristEncoderToAngleFactor = ((1.0 / Constants.Falcon500.TICKS_PER_REV) / Constants.Arm.WRIST_GEAR_RATIO) * 360;

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public double lastAngle;
    public double setpoint;
    public double wristPos;
    public double shoulderPos;
    public int periodicCounter = 0;
    private static Arm INSTANCE;
    boolean isLimitSwitchTrue = false;

    public static Arm getInstance()
    {
        return INSTANCE;
    }

    public Arm(ArmIO io) {
        this.io = io;

        currentPose = Constants.ArmPose.STOW;

        SmartDashboard.putNumber("should be", Constants.Arm.SHOULDER_RESTING_ANGLE / Constants.Arm.SHOULDER_TICKS_TO_DEGREES);
        //SmartDashboard.putNumber("constructor current", ShoulderMotor.getSelectedSensorPosition());

        updateKf(Constants.Arm.SHOULDER_KF, Constants.Arm.SHOULDER_RESTING_ANGLE, Constants.Arm.SHOULDER_PEAK_OUTPUT);
        wristPos = getWristAngle();
        shoulderPos = getShoulderAngle();

        INSTANCE = this;
    }

    public void setShoulderAnglePosition(double degrees)
    {
        double pos = degrees / Constants.Arm.SHOULDER_TICKS_TO_DEGREES;

        setpoint = pos;
        updateKf(Constants.Arm.SHOULDER_KF, degrees, Constants.Arm.SHOULDER_PEAK_OUTPUT);

        System.out.println("setting target shoulder: " + pos);
        io.setShoulderPos(pos);
    }

    public void setWristAnglePosition(double degrees)
    {
        double pos = degrees / Constants.Arm.WRIST_TICKS_TO_DEGREES;

        setpoint = pos;
        System.out.println("setting target wrist: " + pos);
        io.setWristPos(pos);
    }

    public double getShoulderAngle() {
        return inputs.shoulderSensorPos * Constants.Arm.SHOULDER_TICKS_TO_DEGREES;
    }

    public double getWristAngle() {
        return inputs.wristSensorPos * Constants.Arm.WRIST_TICKS_TO_DEGREES;
    }

//    public double getShoulderCurrentOutput()
//    {
//        return ShoulderMotor.getMotorOutputPercent();
//    }
//
//    public double getWristCurrentOutput()
//    {
//        return WristMotor.getMotorOutputPercent();
//    }


    /*
    Fix motor scaling the kf based on the setpoint

    kf: the value you want to set the kf to

    results: divides the input kf by the setpoint and multiplies
    the result by 1024
    */
    private void configArmKF(double kf, double peakOutput)
    {
        double pos = setpoint;
        double kF = (kf / pos) * 1023;
        //lastAngle = pos;

        configArmSlot(Constants.Arm.SHOULDER_KP, Constants.Arm.SHOULDER_KI, 0, kF, peakOutput);
    }

    public void SetWristSpeed(double speed)
    {
        io.setWristSpeed(speed);
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
    public void updateKf(double kf, double pos, double peakOutput)
    {
        double newKF = kf * Math.sin(pos * Math.PI / 180f);
        SmartDashboard.putNumber("kf", newKF * 1023);

        configArmKF(newKF, peakOutput);
    }


    public boolean getLimitSwitchOutput(int limitSwitchPort)
    {
        if(limitSwitchPort == Constants.Arm.SHOULDER_LIMIT_SWITCH_FRONT)
        {
            return inputs.shoulderFwdSwitchClosed == 1;
        }
        else if(limitSwitchPort == Constants.Arm.SHOULDER_LIMIT_SWITCH_BACK)
        {
            return inputs.shoulderRevSwitchClosed == 1;
        }
        else if(limitSwitchPort == Constants.Arm.WRIST_LIMIT_SWITCH)
        {
            return inputs.wristIsSwitchClosed == 1;
        }
        else
        {
            System.out.println("Incorrect limit switch constant");
            return false;
        }
    }

    public void SetPercentOutput(double percent) {

        io.setShoulderPercentOutput(percent);

    }

    public void setShoulderBrakeMode(boolean enabled) {
        io.setShoulderBrakeMode(enabled);
    }

    public void setWristBrakeMode(boolean enabled) {
        io.setWristBrakeMode(enabled);
    }

    public void configArmSlot(double kP, double kI, double kD, double kF, double peakOutput) {
        io.configArmSlot(kP, kI, kD, kF, peakOutput);
    }

    public void SetLightMode(int mode) {
        io.setLightMode(mode);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        periodicCounter = (periodicCounter + 1) % 5;

        if (periodicCounter == 0) {
            wristPos = getWristAngle();
        }
        if (periodicCounter == 1) {
            shoulderPos = getShoulderAngle();
        }

        if(Math.abs(shoulderPos - lastAngle) > Constants.Arm.SHOULDER_ANGLE_UPDATE && periodicCounter == 2)
        {
            lastAngle = shoulderPos;
            updateKf(Constants.Arm.SHOULDER_KF, shoulderPos, Constants.Arm.SHOULDER_PEAK_OUTPUT);
        }
        if(wristPos >= Constants.Arm.MAX_WRIST_ROTATION)
        {
            System.out.println("WRIST IS NOT IN GOOD POS, SENDING TO CARRY");
            setWristAnglePosition(Constants.ArmPos.CARRY_WRIST);
        }

    }
    public Constants.ArmPose getCurrentPose(){
        return currentPose;
    }
    public void setCurrentPosition(Constants.ArmPose newPosition){

        currentPose = newPosition;
    }

}
