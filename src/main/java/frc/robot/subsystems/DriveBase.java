package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;

import java.util.function.Supplier;
//import frc.team1983.constants.RobotMap;

public class DriveBase extends SubsystemBase {

public Pose3d position;
private DifferentialDriveOdometry m_odometry;
    private final static DriveBase INSTANCE = new DriveBase();

    @SuppressWarnings("WeakerAccess")
    public static DriveBase getInstance() {
        return INSTANCE;
    }

public Supplier<Pose2d> currentPose;
    public RamseteController ramseteController;
    public double trackWidthMeters;


    public final TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.getInstance().kMaxSpeedMetersPerSecond,
                    Constants.getInstance().kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.getInstance().kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(Constants.getInstance().autoVoltageConstraint);


    //private CANSparkMax
    private TalonFX leftFrontMotorController;
    private TalonFX rightFrontMotorController;
    private TalonFX leftBackMotorController;
    private TalonFX rightBackMotorController;
    private AHRS navX;

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

    public DriveBase()
    {

        m_odometry =
                new DifferentialDriveOdometry(
                        navX.getRotation2d(), leftFrontMotorController.getSelectedSensorPosition(), rightFrontMotorController.getSelectedSensorPosition());


        leftFrontMotorController=new TalonFX(Constants.getInstance().leftFrontMotorController);
        rightFrontMotorController=new TalonFX(Constants.getInstance().rightFrontMotorController);
        leftBackMotorController=new TalonFX(Constants.getInstance().leftBackMotorController);
        rightBackMotorController=new TalonFX(Constants.getInstance().rightBackMotorController);
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


}

