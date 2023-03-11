package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.autos.SmartDriveCommand;
import frc.robot.constants.Constants;

public class Drivebase implements Subsystem {

    private static Drivebase OGDrivebase;

    DigitalOutput frontRangeSensorTrigger = new DigitalOutput(Constants.Drivebase.FRONT_RANGE_SENSOR_OUTPUT_CHANNEL);

    DigitalOutput backRangeSensorTrigger = new DigitalOutput(Constants.Drivebase.BACK_RANGE_SENSOR_OUTPUT_CHANNEL);

    AnalogInput frontRangeSensorValue = new AnalogInput(Constants.Drivebase.FRONT_RANGE_SENSOR_INPUT_CHANNEL);

    AnalogInput backRangeSensorValue = new AnalogInput(Constants.Drivebase.BACK_RANGE_SENSOR_INPUT_CHANNEL);
    TalonFX leftMotor1 = new TalonFX(Constants.Wobbles.LEFT_MOTOR_1);
    TalonFX leftMotor2 = new TalonFX(Constants.Wobbles.LEFT_MOTOR_2);
    TalonFX rightMotor1 = new TalonFX(Constants.Wobbles.RIGHT_MOTOR_1);
    TalonFX rightMotor2 = new TalonFX(Constants.Wobbles.RIGHT_MOTOR_2);

    int i=0;

    DifferentialDriveKinematics kinematics;

    public RamseteController ramseteController;

    private DifferentialDriveOdometry odometry;

    private DifferentialDrivePoseEstimator poseEstimator;

    private final double TicksPerFoot =
            Constants.Wobbles.TICKS_PER_MOTOR_REV*Constants.Drivebase.GEAR_RATIO /
                    (Constants.Drivebase.WHEEL_DIAMETER * Math.PI);


    AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    public enum DriveDirection {FORWARD,BACKWARD,MOTIONLESS}
    DriveDirection driveDirection = DriveDirection.FORWARD;

    public final TrajectoryConstraint autoVoltageConstraint= new MaxVelocityConstraint(Constants.Drivebase.kMaxSpeedMetersPerSecond);
    public DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.Drivebase.kTrackwidthMeters);

    public final TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.Drivebase.kMaxSpeedMetersPerSecond,
                    Constants.Drivebase.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);




    private Drivebase()
    {
        gyro.calibrate();
        rightMotor1.setInverted(true);
        rightMotor2.setInverted(true);
        rightMotor2.follow(rightMotor1);
        leftMotor2.follow(leftMotor1);

        rightMotor1.setSelectedSensorPosition(0);
        leftMotor1.setSelectedSensorPosition(0);

        kinematics = new DifferentialDriveKinematics(Constants.Drivebase.kTrackwidthMeters);
        ramseteController = new RamseteController();
        odometry =
                new DifferentialDriveOdometry(
                        gyro.getRotation2d(), leftMotor1.getSelectedSensorPosition(), rightMotor1.getSelectedSensorPosition());

        poseEstimator = new DifferentialDrivePoseEstimator(kinematics,gyro.getRotation2d(), ticksToMeters(leftMotor1.getSelectedSensorPosition()), ticksToMeters(rightMotor1.getSelectedSensorPosition()), new Pose2d(8.27,.4191,new Rotation2d(Math.PI /2)));

        CommandScheduler.getInstance().registerSubsystem(this);


    }

        public void runMotor(double turnSpeedLeft, double turnSpeedRight)
    {
        leftMotor1.set(TalonFXControlMode.PercentOutput, turnSpeedLeft);
        rightMotor1.set(TalonFXControlMode.PercentOutput, turnSpeedRight);
    }

    public Pose2d GetCurrentPose(){return odometry.getPoseMeters();}

        public double getPosLeft()
    {
        return leftMotor1.getSelectedSensorPosition()/TicksPerFoot;
    }


        public double getPosRight()
    {
        return -(rightMotor1.getSelectedSensorPosition()/TicksPerFoot);
    }


        public double getHeading()
    {
        return gyro.getAngle();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

        public boolean isCalibrating()
    {
        return gyro.isCalibrating();
    }


        public double getTicksLeft() {

        return leftMotor1.getSelectedSensorPosition();
    }

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


        public double getSpeedLeft()
    {
        return leftMotor1.getSelectedSensorVelocity();
    }

        public double getSpeedRight()
    {
        return (-rightMotor1.getSelectedSensorVelocity());
    }

    public static Drivebase GetDrivebase() {

        if (OGDrivebase == null) {
            OGDrivebase = new Drivebase();
        }

        return OGDrivebase;

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

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        updateOdometry();//update odometry is just backup
        poseEstimator.update(gyro.getRotation2d(),ticksToMeters((int)leftMotor1.getSelectedSensorPosition()), ticksToMeters((int)rightMotor1.getSelectedSensorPosition()));
        SmartDashboard.putNumber("x",poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("y",poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("meters",ticksToMeters((int)leftMotor1.getSelectedSensorPosition()));
            //SmartDashboard.putNumber("x",odometry.getPoseMeters().getX());
            //SmartDashboard.putNumber("y",odometry.getPoseMeters().getY());
                //SmartDashboard.putNumber("x",leftMotor1.getSelectedSensorPosition());
                //SmartDashboard.putNumber("y",rightMotor1.getSelectedSensorPosition());
                //System.out.println(leftMotor1.getSelectedSensorPosition()+","+(rightMotor1.getSelectedSensorPosition());
        //System.out.println(odometry.getPoseMeters().getX()+","+odometry.getPoseMeters().getY());
        //TODO: if we have an april tag in view, call addVisionMeasurement();

    }

    public void updateOdometry() {

        odometry.update(
                gyro.getRotation2d(), ticksToMeters((int)leftMotor1.getSelectedSensorPosition()), ticksToMeters((int)rightMotor1.getSelectedSensorPosition()));

    }

    public double getPitch()
    {
        return gyro.getPitch();
    }

    public void setRightMeters(double meters){
        rightMotor1.set(ControlMode.Velocity,meters);
        rightMotor2.set(ControlMode.Velocity,meters);
    }
    public void setLeftMeters(double meters){
        leftMotor1.set(ControlMode.Velocity,meters);
        leftMotor2.set(ControlMode.Velocity,meters);
    }


    public void setSpeedChassis(ChassisSpeeds chassisSpeeds){

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        System.out.println(wheelSpeeds.leftMetersPerSecond+" , "+wheelSpeeds.rightMetersPerSecond);
        setLeftMeters(wheelSpeeds.leftMetersPerSecond);
        setRightMeters(wheelSpeeds.rightMetersPerSecond);

    }

    double ticksToMeters(double ticks){return ticksToFeet(ticks)/Constants.Drivebase.FEET_PER_METER;}

    double ticksToFeet(double ticks){

        //ticks to feet ticks/ticks_per_rotation*gearratio*wheeldiamater=feet
        return ticks/Constants.Drivebase.TICKS_PER_ROTATION
                /Constants.Drivebase.GEAR_RATIO*Constants.Drivebase.WHEEL_DIAMETER*Math.PI;

    }
    double feetToTicks(double feet){

        //ticks to feet ticks/ticks_per_rotation*gearratio*wheeldiamater=feet
        return feet*Constants.Drivebase.TICKS_PER_ROTATION
                *Constants.Drivebase.GEAR_RATIO/Constants.Drivebase.WHEEL_DIAMETER/Math.PI;

    }

    public double metersToTicks(double meters){return feetToTicks(meters)*Constants.Drivebase.FEET_PER_METER;}

}
