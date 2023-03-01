package frc.robot.subsystems.multidrivebase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.constants.Constants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Drivebase extends SubsystemBase
{
//    TalonFX leftMotor1 = new TalonFX(Constants.FourMotorFalcon500.LEFT_MOTOR_1_DEVICE_NUMBER);
//    TalonFX leftMotor2 = new TalonFX(Constants.FourMotorFalcon500.LEFT_MOTOR_2_DEVICE_NUMBER);
//    TalonFX rightMotor1 = new TalonFX(Constants.FourMotorFalcon500.RIGHT_MOTOR_1_DEVICE_NUMBER);
//    TalonFX rightMotor2 = new TalonFX(Constants.FourMotorFalcon500.RIGHT_MOTOR_2_DEVICE_NUMBER);
//
    //private final double TicksPerFoot = Constants.Falcon500.TICKS_PER_REV * Constants.Drivebase.GEAR_RATIO / (Constants.Drivebase.WHEEL_DIAMETER * Math.PI);

    //AHRS gyro = new AHRS(I2C.Port.kOnboard);

    public Drivebase()
    {

    }
    public abstract void runMotor(double turnSpeedLeft, double turnSpeedRight);
//    {
//        leftMotor1.set(TalonFXControlMode.PercentOutput, turnSpeedLeft);
//        leftMotor2.set(TalonFXControlMode.PercentOutput, turnSpeedLeft);
//        rightMotor1.set(TalonFXControlMode.PercentOutput, -turnSpeedRight);
//        rightMotor2.set(TalonFXControlMode.PercentOutput, -turnSpeedRight);
//    }


    public abstract double getPosLeft();
//    {
//        return leftMotor1.getSelectedSensorPosition()/TicksPerFoot;
//    }

    public abstract double getPosRight();
//    {
//        return -(rightMotor1.getSelectedSensorPosition()/TicksPerFoot);
//    }

    public abstract double getHeading();
//    {
//        return gyro.getAngle();
//    }

    public abstract boolean isCalibrating();
//    {
//        return gyro.isCalibrating();
//    }

    public abstract double getTicksLeft();
    //{return leftMotor1.getSelectedSensorPosition();}

    public abstract void SetBrakeMode(boolean enable);
//    {
//
//    }

    public abstract double getSpeedLeft();
//    {
//        return leftMotor1.getSelectedSensorVelocity();
//    }

    public abstract double getSpeedRight();
//    {
//        return (-rightMotor1.getSelectedSensorVelocity());
//    }

    public abstract void resetGyro();

    public abstract void calibrateGyro();
}

