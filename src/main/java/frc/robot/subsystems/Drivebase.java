package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.constants.Constants;
import frc.robot.constants.DrivebaseConstants;

import java.util.function.Supplier;
//import frc.team1983.constants.RobotMap;

public class Drivebase extends SubsystemBase {

public Pose3d position;

double lastSpeedLeft;

double lastSpeedRight;
private DifferentialDriveOdometry m_odometry;
    private final static Drivebase INSTANCE = new Drivebase();

    @SuppressWarnings("WeakerAccess")
    public static Drivebase getInstance() {
        return INSTANCE;
    }

public Supplier<Pose2d> currentPose;
    public RamseteController ramseteController;
    public double trackWidthMeters;


    public final TrajectoryConfig config =
            new TrajectoryConfig(
                    DrivebaseConstants.MAX_SPEED_METERS_PER_SECOND,
                    DrivebaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(DrivebaseConstants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(DrivebaseConstants.AUTO_VOLTAGE_CONSTRAINT);


    //private CANSparkMax
    private TalonFX leftFrontMotorController;
    private TalonFX rightFrontMotorController;
    private TalonFX leftBackMotorController;
    private TalonFX rightBackMotorController;
    private AHRS navX;

    DifferentialDriveKinematics kinematics;

public double startingPitch =0;

    private double p;

    private double maxSpeed;

    int ticksPerRevolution=(int)(2048);  //on the first wobbles (talonSRX) it was 256*1.333
    float revolutionsPerMeter=(float)6.5616797;

    /**
     * Creates a new instance of this DriveBase. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */

    public Drivebase()
    {

        kinematics = new DifferentialDriveKinematics(DrivebaseConstants.MAX_SPEED_METERS_PER_SECOND);

        m_odometry =
                new DifferentialDriveOdometry(
                        navX.getRotation2d(), leftFrontMotorController.getSelectedSensorPosition(), rightFrontMotorController.getSelectedSensorPosition());


        leftFrontMotorController=new TalonFX(Constants.FourMotorFalcon500.LEFT_MOTOR_1_DEVICE_NUMBER);
        rightFrontMotorController=new TalonFX(Constants.FourMotorFalcon500.RIGHT_MOTOR_1_DEVICE_NUMBER);
        leftBackMotorController=new TalonFX(Constants.FourMotorFalcon500.LEFT_MOTOR_2_DEVICE_NUMBER);
        rightBackMotorController=new TalonFX(Constants.FourMotorFalcon500.RIGHT_MOTOR_2_DEVICE_NUMBER);
        navX = new AHRS(I2C.Port.kMXP);
        navX.calibrate();
        while(navX.isCalibrating());//you problbobly should not do this but it works I guess.
        navX.getAngle();
        startingPitch=getPitch();
        System.out.println(navX.getPitch() +" Pitch at start(startingPitch)*****************************");

                //rightMotorController.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder,0,100);
                //rightMotorController.configSelectedFeedbackCoefficient(ticksToMeters(1));
                //rightMotorController.setInverted(true);

                //leftMotorController.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder,0,100);
                //leftMotorController.configSelectedFeedbackCoefficient(ticksToMeters(1));


    }

    public void setSpeedChassis(ChassisSpeeds chassisSpeeds){

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);


        var leftAcc=wheelSpeeds.leftMetersPerSecond-lastSpeedLeft;
        var rightAcc=wheelSpeeds.leftMetersPerSecond-lastSpeedLeft;


        setLeftMeters(wheelSpeeds.leftMetersPerSecond,leftAcc);
        setRightMeters(wheelSpeeds.rightMetersPerSecond,rightAcc);

        lastSpeedLeft=wheelSpeeds.leftMetersPerSecond;
        lastSpeedRight=wheelSpeeds.rightMetersPerSecond;

    }
    public void setRightMeters(double speed,double acceleration){
        var s=speed*ticksPerRevolution*revolutionsPerMeter;
        var a= acceleration*ticksPerRevolution*revolutionsPerMeter;
        SimpleMotorFeedforward feed=new SimpleMotorFeedforward(DrivebaseConstants.KS,DrivebaseConstants.KV,DrivebaseConstants.KA);
        double f = feed.calculate(speed,acceleration);
        rightFrontMotorController.set(ControlMode.Velocity,s, DemandType.ArbitraryFeedForward,f);
        rightBackMotorController.set(ControlMode.Velocity,s);
    }
    public void setLeftMeters(double speed,double acceleration){
        var s=speed*ticksPerRevolution*revolutionsPerMeter;
        var a= acceleration*ticksPerRevolution*revolutionsPerMeter;
        SimpleMotorFeedforward feed=new SimpleMotorFeedforward(DrivebaseConstants.KS,DrivebaseConstants.KV,DrivebaseConstants.KA);
        double f = feed.calculate(speed,acceleration);
        leftFrontMotorController.set(ControlMode.Velocity,s, DemandType.ArbitraryFeedForward,f);
        leftBackMotorController.set(ControlMode.Velocity,s);
    }




    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                navX.getRotation2d(), leftFrontMotorController.getSelectedSensorPosition(), rightFrontMotorController.getSelectedSensorPosition());
    }
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public int metersToticks(float meters){return (int)(meters*ticksPerRevolution*revolutionsPerMeter);}
    public void reCalibrateNavX() {
        navX.calibrate();
    }

    public boolean isCalibrated(){System.out.println("This probably does not work correctly");return navX.isCalibrating();}//not correct I think.
    public double getRotation(){return navX.getAngle();}//gets as /360 - continuous
    public double getPitch(){
        //System.out.println(navX.getYaw()+": yaw        "+navX.getPitch()+": pitch       "+navX.getRoll()+": roll");


        return navX.getRoll()-startingPitch;}//gets as /360 - continuous


    public float ticksToMeters(int ticks){return (float)(ticks/ticksPerRevolution/revolutionsPerMeter);}

    public void setLeft(double speed){
        leftFrontMotorController.set(ControlMode.PercentOutput,-speed);
        leftBackMotorController.set(ControlMode.PercentOutput,-speed);
    }
    public void setRight(double speed){
        rightFrontMotorController.set(ControlMode.PercentOutput,speed);
        rightBackMotorController.set(ControlMode.PercentOutput,speed);
    }





    public int getLeftTicks(){return (int)leftFrontMotorController.getSelectedSensorPosition();}//always front
    public int getRightTicks(){return (int)rightFrontMotorController.getSelectedSensorPosition();}//always front

    public double getLeftMeters(){return ticksToMeters(getLeftTicks());}
    public double getRightMeters(){return ticksToMeters(getRightTicks());}


    /*public void SetPidDistance(float meters){
        leftMotorController.set(TalonSRXControlMode.Position, metersToticks(meters));
        rightMotorController.set(TalonSRXControlMode.Position, metersToticks(meters));
        // rightMotorController.set(TalonSRXControlMode.Follower, 3);

    }*/
    /*public void setSlotLeft(SlotConfiguration s){
        leftMotorController.configureSlot(s);

    }
    public void SetTargetDistanceRight(float distance){

        rightMotorController.set(TalonSRXControlMode.Position,distance);
    }*/


//dummy values
    public double getPosLeft(){return 1;}

    public double getPosRight(){return 1;}
    public double getHeading(){return 1;}

    public void runMotor(double a,double b){}

    public double getSpeedLeft(){return 1;}

    public double getSpeedRight(){return 1;}

    public int getTicksLeft(){return 1;}



}

