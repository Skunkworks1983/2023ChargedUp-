package frc.robot.subsystems.multidrivebase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class Drivebase4MotorTalonFX extends Drivebase {

    private static Drivebase OGDrivebase;
    private TalonFX leftMotor1;
    private TalonFX rightMotor1;
    private TalonFX leftMotor2;
    private TalonFX rightMotor2;


    private Drivebase4MotorTalonFX() {

        leftMotor1 = new TalonFX(Constants.MultiDrivebase.Robot2022.LEFT_MOTOR_1);
        rightMotor1 = new TalonFX(Constants.MultiDrivebase.Robot2022.RIGHT_MOTOR_1);
        leftMotor2.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);

    }

    public void SetSpeed(double leftSpeed, double rightSpeed) {

        leftMotor1.set(TalonFXControlMode.PercentOutput, leftSpeed);
        rightMotor1.set(TalonFXControlMode.PercentOutput, -rightSpeed);
    }

    public double GetLeftDistance() {

        return -leftMotor1.getSelectedSensorPosition() / Constants.MultiDrivebase.Robot2022.TICKS_PER_FOOT;

    }

    public double GetRightDistance() {

        return rightMotor1.getSelectedSensorPosition() / Constants.MultiDrivebase.Robot2022.TICKS_PER_FOOT;

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

    public static Drivebase GetDrivebase() {

        if (OGDrivebase == null) {
            OGDrivebase = new Drivebase4MotorTalonFX();
        }
        return OGDrivebase;

    }
}
