package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;

import static java.lang.Double.NaN;

public class Drivebase implements Subsystem {

    private static Drivebase OGDrivebase;
    TalonFX leftMotor1 = new TalonFX(Constants.Wobbles.LEFT_MOTOR_1);
    TalonFX leftMotor2 = new TalonFX(Constants.Wobbles.LEFT_MOTOR_2);
    TalonFX rightMotor1 = new TalonFX(Constants.Wobbles.RIGHT_MOTOR_1);
    TalonFX rightMotor2 = new TalonFX(Constants.Wobbles.RIGHT_MOTOR_2);

    double backRangeVoltage;

    double frontRangeVoltage;

    private double lastHeading;

    public double getFrontRangeVoltage() {
        return frontRangeVoltage;
    }

    public double getBackRangeVoltage() {
        return backRangeVoltage;
    }

    public void setFrontRangeVoltage(double voltage) {
        frontRangeVoltage = voltage;
    }

    public void setBackRangeVoltage(double voltage) {
        backRangeVoltage = voltage;
    }

    DigitalOutput frontRangeSensorTrigger = new DigitalOutput(Constants.Drivebase.FRONT_RANGE_SENSOR_OUTPUT_CHANNEL);

    DigitalOutput backRangeSensorTrigger = new DigitalOutput(Constants.Drivebase.BACK_RANGE_SENSOR_OUTPUT_CHANNEL);

    AnalogInput frontRangeSensorValue = new AnalogInput(Constants.Drivebase.FRONT_RANGE_SENSOR_INPUT_CHANNEL);

    AnalogInput backRangeSensorValue = new AnalogInput(Constants.Drivebase.BACK_RANGE_SENSOR_INPUT_CHANNEL);

    public enum DriveDirection {FORWARD, BACKWARD, MOTIONLESS, UNCLEAR}

    DriveDirection driveDirection = DriveDirection.FORWARD;

    public void setCurrentDirection(Drivebase.DriveDirection direction) {
        driveDirection = direction;
    }

    private boolean isHeadingReliable = false;

    private final double TicksPerFoot =
            Constants.Wobbles.TICKS_PER_MOTOR_REV * Constants.Drivebase.GEAR_RATIO /
                    (Constants.Drivebase.WHEEL_DIAMETER * Math.PI);

    AHRS gyro = new AHRS(SPI.Port.kMXP);

    Timer timer = new Timer();

    private Drivebase() {
        gyro.calibrate();
    }

    public void runMotor(double turnSpeedLeft, double turnSpeedRight) {
        leftMotor1.set(TalonFXControlMode.PercentOutput, turnSpeedLeft);
        leftMotor2.set(TalonFXControlMode.PercentOutput, turnSpeedLeft);
        rightMotor1.set(TalonFXControlMode.PercentOutput, -turnSpeedRight);
        rightMotor2.set(TalonFXControlMode.PercentOutput, -turnSpeedRight);
        if (turnSpeedLeft > 0 && turnSpeedRight > 0) driveDirection = DriveDirection.FORWARD;
        else if (turnSpeedLeft < 0 && turnSpeedRight < 0) driveDirection = DriveDirection.BACKWARD;
        else {
            driveDirection = DriveDirection.UNCLEAR;
        }
    }

    public double getPosLeft() {
        return leftMotor1.getSelectedSensorPosition() / TicksPerFoot;
    }


    public double getPosRight() {
        return -(rightMotor1.getSelectedSensorPosition() / TicksPerFoot);
    }


    public double getHeading() {

        if (isHeadingReliable) {

            return gyro.getAngle();

        } else {

            return NaN;

        }
    }


    public double getPitch() {
        return gyro.getPitch();
    }

    public boolean isCalibrating() {
        return gyro.isCalibrating();
    }


    public double getTicksLeft() {
        return leftMotor1.getSelectedSensorPosition();
    }

    public void SetBrakeMode(boolean enable) {
        if (enable) {

            leftMotor1.setNeutralMode(NeutralMode.Brake);
            rightMotor1.setNeutralMode(NeutralMode.Brake);
            leftMotor2.setNeutralMode(NeutralMode.Brake);
            rightMotor2.setNeutralMode(NeutralMode.Brake);

        } else {

            leftMotor1.setNeutralMode(NeutralMode.Coast);
            rightMotor1.setNeutralMode(NeutralMode.Coast);
            leftMotor2.setNeutralMode(NeutralMode.Coast);
            rightMotor2.setNeutralMode(NeutralMode.Coast);
        }
    }


    public double getSpeedLeft() {
        return leftMotor1.getSelectedSensorVelocity();
    }

    public double getSpeedRight() {
        return (-rightMotor1.getSelectedSensorVelocity());
    }

    public void waitForHeadingReliable() {

        System.out.println("waitForHeadingReliable method is called");

        timer.start();

        while (gyro.isCalibrating()) {

            if (timer.hasElapsed(Constants.Drivebase.WAIT_TIME_FOR_GYRO_CALIBRATION)) {

                System.out.println("gyro took too long to calibrate");
                System.out.println("heading reliability is " + isHeadingReliable);

                return;
            }
        }


        isHeadingReliable = true;

        System.out.println("gyro finished calibrating");
        System.out.println("heading reliability is " + isHeadingReliable);

    }

    public void setGyroStatus(boolean status) {
        isHeadingReliable = status;
    }


    @Override
    public void periodic() {
        if (isHeadingReliable) {
            if (gyro.isCalibrating() || !gyro.isConnected()) {

                isHeadingReliable = false;

                System.out.println("GYRO CRASHED!!! - GYRO IS NOT CALIBRATED OR CONNECTED");
            }

            if (Math.abs(getHeading() - lastHeading) >= Constants.Drivebase.HEADING_TOO_BIG) {

                isHeadingReliable = false;

                System.out.println("THE GYROSCOPE HAS CRASHED!!! - HEADING IS TOO LARGE");
            }

            lastHeading = getHeading();
        }
    }

    public DriveDirection getDriveDirection() {
        return driveDirection;
    }

    public int getFrontRangeSensor() {
        return frontRangeSensorValue.getValue();
    }

    public int getBackRangeSensor() {
        return backRangeSensorValue.getValue();
    }

    public int getDirectionRangeSensor() {
        if (driveDirection == DriveDirection.FORWARD) return getFrontRangeSensor();
        if (driveDirection == DriveDirection.BACKWARD) return getBackRangeSensor();
        return 0;
    }

    public void setDirectionRangeSensor(boolean value) {
        if (driveDirection == DriveDirection.FORWARD) setFrontRangeSensor(value);
        if (driveDirection == DriveDirection.BACKWARD) setBackRangeSensor(value);
    }

    public void setBackRangeSensor(boolean value) {
        backRangeSensorTrigger.set(value);
    }

    public void setFrontRangeSensor(boolean value) {
        frontRangeSensorTrigger.set(value);
    }

    public static Drivebase GetDrivebase() {

        if (OGDrivebase == null) {
            OGDrivebase = new Drivebase();
        }

        return OGDrivebase;

    }
}
