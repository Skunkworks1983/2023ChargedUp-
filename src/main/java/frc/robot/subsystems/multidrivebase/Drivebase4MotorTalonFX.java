package frc.robot.subsystems.multidrivebase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

import static frc.robot.constants.Constants.Drivebase.MAX_SPEED_METERS_PER_SECOND;
import static frc.robot.constants.Constants.Drivebase.TRACK_WIDTH_METERS;

public class Drivebase4MotorTalonFX extends Drivebase {

    private static Drivebase OGDrivebase;
    private TalonFX leftMotor1;
    private TalonFX rightMotor1;
    private TalonFX leftMotor2;
    private TalonFX rightMotor2;

    public Pose3d position;

    private AHRS navX;

    double lastSpeedLeft;

    double lastSpeedRight;

    public RamseteController ramseteController;

    private DifferentialDriveOdometry m_odometry;

    DifferentialDriveKinematics kinematics;


    public static final TrajectoryConstraint AUTO_VOLTAGE_CONSTRAINT= new MaxVelocityConstraint(MAX_SPEED_METERS_PER_SECOND);

    public static final DifferentialDriveKinematics kDriveKinematics= new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    private Drivebase4MotorTalonFX() {


        navX = new AHRS(I2C.Port.kMXP);
        navX.calibrate();
        int i = 0;
        while(navX.isCalibrating()){i++;if(i>200)break;}//about 4 seconds wait
        navX.getAngle();

        leftMotor1 = new TalonFX(Constants.MultiDrivebase.Robot2022.LEFT_MOTOR_1);
        rightMotor1 = new TalonFX(Constants.MultiDrivebase.Robot2022.RIGHT_MOTOR_1);

        leftMotor2 = new TalonFX(Constants.MultiDrivebase.Robot2022.LEFT_MOTOR_2);
        rightMotor2 = new TalonFX(Constants.MultiDrivebase.Robot2022.RIGHT_MOTOR_2);
        double Kp=Constants.Drivebase.DRIVEBASE_KP;
        leftMotor1.config_kP(0,Kp);
        rightMotor1.config_kP(0,Kp);
        leftMotor2.config_kP(0,Kp);
        rightMotor2.config_kP(0,Kp);

        rightMotor1.setInverted(true);
        rightMotor2.setInverted(true);

        leftMotor2.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);

        m_odometry =
                new DifferentialDriveOdometry(
                        navX.getRotation2d(), leftMotor1.getSelectedSensorPosition(), rightMotor1.getSelectedSensorPosition());

        kinematics = new DifferentialDriveKinematics(Constants.Drivebase.MAX_SPEED_METERS_PER_SECOND);
        ramseteController = new RamseteController();
        SetBrakeMode(true);
    }

    public void SetSpeed(double leftSpeed, double rightSpeed) {

        leftMotor1.set(TalonFXControlMode.PercentOutput, leftSpeed);
        rightMotor1.set(TalonFXControlMode.PercentOutput, -rightSpeed);
    }

    public double GetLeftDistance() {

        return -leftMotor1.getSelectedSensorPosition() / Constants.MultiDrivebase.Robot2022.TICKS_PER_FOOT;

    }

    public final TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.Drivebase.TRAJECTORY_MAX_VELOCITY,
                    Constants.Drivebase.TRAJECTORY_MAX_ACCELERATION)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Drivebase4MotorTalonFX.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(AUTO_VOLTAGE_CONSTRAINT);

    public double GetRightDistance() {

        return rightMotor1.getSelectedSensorPosition() / Constants.MultiDrivebase.Robot2022.TICKS_PER_FOOT;

    }

    public void periodic() {
        // Update the odometry in the periodic block
//        System.out.println(navX.getYaw());
        m_odometry.update(
                navX.getRotation2d(), leftMotor1.getSelectedSensorPosition(), rightMotor1.getSelectedSensorPosition());
    }

    public void end(){



    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    public void setSpeedChassis(ChassisSpeeds chassisSpeeds){

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        SmartDashboard.putNumber("curve left", wheelSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("curve right", wheelSpeeds.leftMetersPerSecond);


        var leftAcc=wheelSpeeds.leftMetersPerSecond-lastSpeedLeft;
        var rightAcc=wheelSpeeds.leftMetersPerSecond-lastSpeedLeft;

System.out.println("left and right meters per second"+wheelSpeeds.leftMetersPerSecond+","+wheelSpeeds.rightMetersPerSecond);
        double mult = 2;
        setLeftMeters(wheelSpeeds.leftMetersPerSecond*mult,leftAcc);
        setRightMeters(wheelSpeeds.rightMetersPerSecond*mult,rightAcc);

        lastSpeedLeft=wheelSpeeds.leftMetersPerSecond;
        lastSpeedRight=wheelSpeeds.rightMetersPerSecond;

    }
    public void setRightMeters(double speed,double acceleration){
        var s=speed*Constants.MultiDrivebase.Wobbles.TICKS_PER_METER*.1;
        var a= acceleration*Constants.MultiDrivebase.Wobbles.TICKS_PER_METER;
        //SimpleMotorFeedforward feed=new SimpleMotorFeedforward(Constants.Drivebase.KS,Constants.Drivebase.KV,Constants.Drivebase.KA);
        //double f = feed.calculate(speed,acceleration);
        //rightMotor1.set(ControlMode.Velocity,s, DemandType.ArbitraryFeedForward,f);
        rightMotor1.set(ControlMode.Velocity,s);
        rightMotor2.set(ControlMode.Velocity,s);
        //System.out.println("ticks/.1s right: "+s);

        //rightMotor2.set(ControlMode.Velocity,s);
    }
    public void setLeftMeters(double speed,double acceleration){

        var s=speed*Constants.MultiDrivebase.Wobbles.TICKS_PER_METER*.1;

        var a= acceleration*Constants.MultiDrivebase.Wobbles.TICKS_PER_METER;
        SimpleMotorFeedforward feed=new SimpleMotorFeedforward(Constants.Drivebase.KS,Constants.Drivebase.KV, Constants.Drivebase.KA);
        double f = feed.calculate(speed,acceleration);
        //leftMotor1.set(ControlMode.Velocity,s, DemandType.ArbitraryFeedForward,f);
        leftMotor1.set(ControlMode.Velocity,s);//s
        leftMotor2.set(ControlMode.Velocity,s);//s
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
