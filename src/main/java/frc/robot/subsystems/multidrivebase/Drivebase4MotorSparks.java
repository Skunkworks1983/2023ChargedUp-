package frc.robot.subsystems.multidrivebase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class Drivebase4MotorSparks extends Drivebase {


private static Drivebase OGDrivebase;
    public Drivebase4MotorSparks ()
    {

        leftMotor1.getEncoder().setPositionConversionFactor(1/Constants.MultiDrivebase.Robot2020.TICKS_PER_FOOT);
        rightMotor1.getEncoder().setPositionConversionFactor(1/Constants.MultiDrivebase.Robot2020.TICKS_PER_FOOT);
        leftMotor2.getEncoder().setPositionConversionFactor(1/Constants.MultiDrivebase.Robot2020.TICKS_PER_FOOT);
        rightMotor2.getEncoder().setPositionConversionFactor(1/Constants.MultiDrivebase.Robot2020.TICKS_PER_FOOT);

    }

        private CANSparkMax leftMotor1 = new CANSparkMax(Constants.MultiDrivebase.Robot2020.LEFT_MOTOR_1, MotorType.kBrushless);
        private CANSparkMax rightMotor1 = new CANSparkMax(Constants.MultiDrivebase.Robot2020.RIGHT_MOTOR_1, MotorType.kBrushless);
        private CANSparkMax leftMotor2 = new CANSparkMax(Constants.MultiDrivebase.Robot2020.LEFT_MOTOR_2, MotorType.kBrushless);
        private CANSparkMax rightMotor2 = new CANSparkMax(Constants.MultiDrivebase.Robot2020.RIGHT_MOTOR_2, MotorType.kBrushless);



    public void SetSpeed(double leftSpeed, double rightSpeed) {

        leftMotor1.set(leftSpeed);
        rightMotor1.set(rightSpeed);
        leftMotor2.set(leftSpeed);
        rightMotor2.set(rightSpeed);
    }

    public double GetLeftDistance() {

        leftMotor1.getEncoder();

        return -leftMotor1.getEncoder().getPosition() / Constants.MultiDrivebase.Robot2022.TICKS_PER_FOOT;
    }


    public double GetRightDistance() {

        rightMotor1.getEncoder();

        return -leftMotor1.getEncoder().getPosition() / Constants.MultiDrivebase.Robot2022.TICKS_PER_FOOT;

    }

    public void SetBrakeMode(boolean enable) {
        if (enable) {

            leftMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
            rightMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
            leftMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
            rightMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        } else {

            leftMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
            rightMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
            leftMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
            rightMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);

        }

    }
    public static Drivebase GetDrivebase() {

        if (OGDrivebase == null) {
            OGDrivebase = new Drivebase4MotorSparks();
        }
        return OGDrivebase;

    }
}
