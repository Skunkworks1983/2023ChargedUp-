package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;

public class Drivebase implements Subsystem {

    private static Drivebase OGDrivebase;
    TalonFX leftMotor1 = new TalonFX(Constants.Wobbles.LEFT_MOTOR_1);
    TalonFX leftMotor2 = new TalonFX(Constants.Wobbles.LEFT_MOTOR_2);
    TalonFX rightMotor1 = new TalonFX(Constants.Wobbles.RIGHT_MOTOR_1);
    TalonFX rightMotor2 = new TalonFX(Constants.Wobbles.RIGHT_MOTOR_2);

    private final double TicksPerFoot =
            Constants.Wobbles.TICKS_PER_MOTOR_REV * Constants.Drivebase.GEAR_RATIO /
                    (Constants.Drivebase.WHEEL_DIAMETER * Math.PI);

    AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    private Drivebase() {
        gyro.calibrate();
    }

    public void runMotor(double turnSpeedLeft, double turnSpeedRight) {
        leftMotor1.set(TalonFXControlMode.PercentOutput, turnSpeedLeft);
        leftMotor2.set(TalonFXControlMode.PercentOutput, turnSpeedLeft);
        rightMotor1.set(TalonFXControlMode.PercentOutput, -turnSpeedRight);
        rightMotor2.set(TalonFXControlMode.PercentOutput, -turnSpeedRight);
    }


    public double getPosLeft() {
        return leftMotor1.getSelectedSensorPosition() / TicksPerFoot;
    }


    public double getPosRight() {
        return -(rightMotor1.getSelectedSensorPosition() / TicksPerFoot);
    }


    public double getHeading() {
        return gyro.getAngle();
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

    public static Drivebase GetDrivebase() {

        if (OGDrivebase == null) {
            OGDrivebase = new Drivebase();
        }

        return OGDrivebase;

    }
}
