package frc.robot.subsystems.multidrivebase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.constants.Constants;

public class Drivebase4MotorTalonFX extends Drivebase {

    private static Drivebase OGDrivebase;
    TalonFX leftMotor1 = new TalonFX(Constants.Wobbles.LEFT_MOTOR_1);
    TalonFX leftMotor2 = new TalonFX(Constants.Wobbles.LEFT_MOTOR_2);
    TalonFX rightMotor1 = new TalonFX(Constants.Wobbles.RIGHT_MOTOR_1);
    TalonFX rightMotor2 = new TalonFX(Constants.Wobbles.RIGHT_MOTOR_2);

    DigitalOutput frontRangeSensorTrigger = new DigitalOutput(Constants.Drivebase.FRONT_RANGE_SENSOR_OUTPUT_CHANNEL);

    DigitalOutput backRangeSensorTrigger = new DigitalOutput(Constants.Drivebase.BACK_RANGE_SENSOR_OUTPUT_CHANNEL);

    AnalogInput frontRangeSensorValue = new AnalogInput(Constants.Drivebase.FRONT_RANGE_SENSOR_INPUT_CHANNEL);

    AnalogInput backRangeSensorValue = new AnalogInput(Constants.Drivebase.FRONT_RANGE_SENSOR_INPUT_CHANNEL);

    public enum DriveDirection {FORWARD,BACKWARD,MOTIONLESS}
    DriveDirection driveDirection = DriveDirection.MOTIONLESS;

    private final double TicksPerFoot =
            Constants.Wobbles.TICKS_PER_MOTOR_REV*Constants.Drivebase.GEAR_RATIO /
                    (Constants.Drivebase.WHEEL_DIAMETER * Math.PI);

    AHRS gyro = new AHRS(I2C.Port.kMXP);

    private Drivebase4MotorTalonFX ()
    {
    }
    @Override
        public void runMotor(double turnSpeedLeft, double turnSpeedRight)
    {
        leftMotor1.set(TalonFXControlMode.PercentOutput, turnSpeedLeft);
        leftMotor2.set(TalonFXControlMode.PercentOutput, turnSpeedLeft);
        rightMotor1.set(TalonFXControlMode.PercentOutput, -turnSpeedRight);
        rightMotor2.set(TalonFXControlMode.PercentOutput, -turnSpeedRight);
    }


    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        updateOdometry();
    }
    public void updateOdometry() {
        odometry.update(
                navX.getRotation2d(), ticksToMeters((int)leftFrontMotorController.getSelectedSensorPosition()), ticksToMeters((int)rightFrontMotorController.getSelectedSensorPosition()));
    }

    public void resetOdometry(){
        leftFrontMotorController.setSelectedSensorPosition(0);

        rightFrontMotorController.setSelectedSensorPosition(0);

        leftBackMotorController.setSelectedSensorPosition(0);

        navX.reset();

        rightBackMotorController.setSelectedSensorPosition(0);
        odometry.resetPosition(new Rotation2d(0),0,0,new Pose2d(0,0,new Rotation2d(0)));

    }

    @Override
        public double getPosLeft()
    {
        return leftMotor1.getSelectedSensorPosition()/TicksPerFoot;
    }

    @Override
        public double getPosRight()
    {
        return -(rightMotor1.getSelectedSensorPosition()/TicksPerFoot);
    }

    @Override
        public double getHeading()
    {
        return gyro.getAngle();
    }

    @Override
        public boolean isCalibrating()
    {
        return gyro.isCalibrating();
    }

    @Override
        public double getTicksLeft() {

        return leftMotor1.getSelectedSensorPosition();
    }
    @Override
        public void SetBrakeMode(boolean enable)
    {
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

    @Override
        public double getSpeedLeft()
    {
        return leftMotor1.getSelectedSensorVelocity();
    }
    @Override
        public double getSpeedRight()
    {
        return (-rightMotor1.getSelectedSensorVelocity());
    }

    public DriveDirection getDriveDirection(){return driveDirection;}

    public int getFrontRangeSensor(){
        return frontRangeSensorValue.getValue();
    }
    public int getBackRangeSensor(){
        return backRangeSensorValue.getValue();
    }

    public int getDirectionRangeSensor(){
        if(driveDirection==DriveDirection.FORWARD)return getFrontRangeSensor();
        if(driveDirection==DriveDirection.BACKWARD)return getBackRangeSensor();
        return 0;
    }

    public void setDirectionRangeSensor(boolean value){
        if(driveDirection==DriveDirection.FORWARD)setFrontRangeSensor(value);
        if(driveDirection==DriveDirection.BACKWARD)setBackRangeSensor(value);
    }

    public void setBackRangeSensor(boolean value){backRangeSensorTrigger.set(value);}

    public void setFrontRangeSensor(boolean value){frontRangeSensorTrigger.set(value);}

    public static Drivebase GetDrivebase() {

        if (OGDrivebase == null) {
            OGDrivebase = new Drivebase4MotorTalonFX();
        }

        return OGDrivebase;

    }
}
