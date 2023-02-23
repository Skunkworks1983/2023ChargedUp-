package frc.robot.subsystems.multidrivebase;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.constants.Constants;

public class Drivebase4MotorSparks extends Drivebase {
private static Drivebase OGDrivebase;

   private CANSparkMax leftMotor1 = new CANSparkMax
           (Constants.Robot2020.LEFT_MOTOR_1, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax rightMotor1 = new CANSparkMax
            (Constants.Robot2020.RIGHT_MOTOR_1, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax leftMotor2 = new CANSparkMax
            (Constants.Robot2020.LEFT_MOTOR_2, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax rightMotor2 = new CANSparkMax
            (Constants.Robot2020.RIGHT_MOTOR_2, CANSparkMaxLowLevel.MotorType.kBrushless);

    AHRS gyro = new AHRS(I2C.Port.kOnboard);

    private final double TicksPerFoot =
            Constants.Robot2020.TICKS_PER_MOTOR_REV*Constants.Drivebase.GEAR_RATIO /
                    (Constants.Drivebase.WHEEL_DIAMETER * Math.PI);

    private Drivebase4MotorSparks () {

        leftMotor1.getEncoder().setPositionConversionFactor(1/Constants.Robot2020.TICKS_PER_FOOT);
        rightMotor1.getEncoder().setPositionConversionFactor(1/Constants.Robot2020.TICKS_PER_FOOT);
        leftMotor2.getEncoder().setPositionConversionFactor(1/Constants.Robot2020.TICKS_PER_FOOT);
        rightMotor2.getEncoder().setPositionConversionFactor(1/Constants.Robot2020.TICKS_PER_FOOT);
    }

    @Override
    public void runMotor(double turnSpeedLeft, double turnSpeedRight) {

        leftMotor1.set(turnSpeedLeft);
        rightMotor1.set(turnSpeedRight);
        leftMotor2.set(turnSpeedLeft);
        rightMotor2.set(turnSpeedRight);
    }

    @Override
    public double getPosLeft() {
        return -leftMotor1.getEncoder().getPosition() / Constants.Robot2022.TICKS_PER_FOOT;
    }

    @Override
    public double getPosRight() {
       return -rightMotor1.getEncoder().getPosition() / Constants.Robot2022.TICKS_PER_FOOT;
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
    public double getTicksLeft()
    {
        return leftMotor1.getEncoder().getPosition();
    }

    @Override
    public void SetBrakeMode(boolean enable)
    {
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

    @Override
    public double getSpeedLeft() {

        return leftMotor1.getEncoder().getVelocity();
    }

    @Override
    public double getSpeedRight() {

        return rightMotor1.getEncoder().getVelocity();
    }

    public static Drivebase GetDrivebase() {

        if (OGDrivebase == null) {
            OGDrivebase = new Drivebase4MotorSparks();
        }

        return OGDrivebase;
    }
}
