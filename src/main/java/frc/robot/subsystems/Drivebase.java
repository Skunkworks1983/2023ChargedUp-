package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.drivebase.ArcadeDrive;
import frc.robot.constants.Constants;
import frc.robot.services.Oi;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static java.lang.Double.NaN;

public class Drivebase implements Subsystem {

    public Command ArcadeDrive = new ArcadeDrive(this, Oi.Instance, LimeLight.getInstance());

    private static Drivebase OGDrivebase;

    DigitalOutput frontRangeSensorTrigger = new DigitalOutput(Constants.Drivebase.FRONT_RANGE_SENSOR_OUTPUT_CHANNEL);

    DigitalOutput backRangeSensorTrigger = new DigitalOutput(Constants.Drivebase.BACK_RANGE_SENSOR_OUTPUT_CHANNEL);

    AnalogInput frontRangeSensorValue = new AnalogInput(Constants.Drivebase.FRONT_RANGE_SENSOR_INPUT_CHANNEL);

    AnalogInput backRangeSensorValue = new AnalogInput(Constants.Drivebase.BACK_RANGE_SENSOR_INPUT_CHANNEL);
    TalonFX leftMotor1 = new TalonFX(Constants.Wobbles.LEFT_MOTOR_1);
    TalonFX leftMotor2 = new TalonFX(Constants.Wobbles.LEFT_MOTOR_2);
    TalonFX rightMotor1 = new TalonFX(Constants.Wobbles.RIGHT_MOTOR_1);
    TalonFX rightMotor2 = new TalonFX(Constants.Wobbles.RIGHT_MOTOR_2);
    Pose2d aprilTag1 = new Pose2d(inchesToMeters(610.77), inchesToMeters(42.19),new Rotation2d(Math.PI));
    Pose2d aprilTag2 = new Pose2d(inchesToMeters(610.77), inchesToMeters(108.19),new Rotation2d(Math.PI));
    Pose2d aprilTag3 = new Pose2d(inchesToMeters(610.77), inchesToMeters(174.19),new Rotation2d(Math.PI));
    double backRangeVoltage;

    double frontRangeVoltage;

    private Arm armInstance = Arm.getInstance();

    public double getFrontRangeVoltage() {
        return frontRangeVoltage;
    }

    public double getBackRangeVoltage() {
        return backRangeVoltage;
    }

    public void setFrontRangeVoltage(double voltage) {
        frontRangeVoltage = voltage;
    }

    public void setBackRangeVoltage(double voltage) {
        backRangeVoltage = voltage;
    }

    public enum DriveDirection {FORWARD, BACKWARD, MOTIONLESS, UNCLEAR}

    DriveDirection driveDirection = DriveDirection.FORWARD;


    private boolean isHeadingReliable;

    private SimpleMotorFeedforward feedforward;

    private int seeTag = 0;
    private boolean poseInitilized;

    DifferentialDriveKinematics kinematics;

    public RamseteController ramseteController;

    private DifferentialDriveOdometry odometry;

    private DifferentialDrivePoseEstimator poseEstimator;

    private final double TicksPerFoot =
            Constants.Wobbles.TICKS_PER_MOTOR_REV * Constants.Drivebase.OLD_GEAR_RATIO /
                    (Constants.Drivebase.WHEEL_DIAMETER * Math.PI);

    //public AHRS gyro = new AHRS(I2C.Port.kMXP);
public AHRS gyro = new AHRS(I2C.Port.kOnboard);

    boolean isRedAlliance;

    public final TrajectoryConstraint autoVoltageConstraint= new MaxVelocityConstraint(Constants.Drivebase.kMaxSpeedMetersPerSecond);
    public DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.Drivebase.kTrackwidthMeters);

    private Timer timer = new Timer();
    public final TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.Drivebase.kMaxSpeedMetersPerSecond,
                    Constants.Drivebase.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint).setReversed(false);

    public final TrajectoryConfig reversedConfig =
            new TrajectoryConfig(
                    Constants.Drivebase.kMaxSpeedMetersPerSecond,
                    Constants.Drivebase.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint).setReversed(true);

private final Field2d field= new Field2d();
public Field2d getField(){
    return field;
}

//private final edu.wpi.first.wpilibj.smartdashboard.FieldObject2d

    private Drivebase()
    {
        SmartDashboard.putData("field",field);
        //setDefaultCommand(ArcadeDrive);
        gyro.calibrate();
        isHeadingReliable = false;
        isRedAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Red;
        System.out.println("drivebase is constructing");
        rightMotor1.setInverted(true);
        rightMotor2.setInverted(true);
        rightMotor2.follow(rightMotor1);
        leftMotor2.follow(leftMotor1);

        rightMotor1.setSelectedSensorPosition(0);
        leftMotor1.setSelectedSensorPosition(0);

        leftMotor1.config_kP(0,Constants.Drivebase.AUTO_KP);
        leftMotor2.config_kP(0,Constants.Drivebase.AUTO_KP);

        rightMotor1.config_kP(0,Constants.Drivebase.AUTO_KP);
        rightMotor2.config_kP(0,Constants.Drivebase.AUTO_KP);
        leftMotor1.config_kD(0,Constants.Drivebase.AUTO_KD);
        leftMotor2.config_kD(0,Constants.Drivebase.AUTO_KD);

        rightMotor1.config_kD(0,Constants.Drivebase.AUTO_KD);
        rightMotor2.config_kD(0,Constants.Drivebase.AUTO_KD);

        leftMotor1.configOpenloopRamp(0,0);

        rightMotor1.configOpenloopRamp(0,0);

        leftMotor2.configOpenloopRamp(0,0);

        rightMotor2.configOpenloopRamp(0,0);

        leftMotor1.config_kF(0,0);

        rightMotor1.config_kF(0,0);

        leftMotor2.config_kF(0,0);

        rightMotor2.config_kF(0,0);

        rightMotor1.setSelectedSensorPosition(0);
        leftMotor1.setSelectedSensorPosition(0);

        kinematics = new DifferentialDriveKinematics(Constants.Drivebase.kTrackwidthMeters);
        ramseteController = new RamseteController();
        ramseteController.setTolerance(new Pose2d(.05,.05,new Rotation2d(Math.PI/20)));
        odometry =
                new DifferentialDriveOdometry(
                        gyro.getRotation2d(), leftMotor1.getSelectedSensorPosition(), rightMotor1.getSelectedSensorPosition());

        poseEstimator = new DifferentialDrivePoseEstimator(
                kinematics,
                gyro.getRotation2d(),
                ticksToMeters(leftMotor1.getSelectedSensorPosition()),
                ticksToMeters(rightMotor1.getSelectedSensorPosition()),
                new Pose2d(0,0,new Rotation2d(0))
        );
        CommandScheduler.getInstance().registerSubsystem(this);


    }

    public void runMotor(double turnSpeedLeft, double turnSpeedRight) {
        leftMotor1.set(TalonFXControlMode.PercentOutput, turnSpeedLeft);
        rightMotor1.set(TalonFXControlMode.PercentOutput, turnSpeedRight);
        if (turnSpeedLeft > 0 && turnSpeedRight > 0) driveDirection = DriveDirection.FORWARD;
        else if (turnSpeedLeft < 0 && turnSpeedRight < 0) driveDirection = DriveDirection.BACKWARD;
        else {
            driveDirection = DriveDirection.UNCLEAR;
        }
    }

    public Pose2d GetCurrentPose(){return poseEstimator.getEstimatedPosition();}

    public double getPosLeft() {
        return leftMotor1.getSelectedSensorPosition() / TicksPerFoot;
    }


    public double getPosRight() {
        return -(rightMotor1.getSelectedSensorPosition() / TicksPerFoot);
    }


    public double getHeading() {

        if (isHeadingReliable) {

            return gyro.getAngle();

        } else {

            return NaN;

        }
    }

    public void setPose(Pose2d pose){
    poseEstimator.resetPosition(
            gyro.getRotation2d(),
            ticksToMeters((int)leftMotor1.getSelectedSensorPosition()),
            ticksToMeters((int)rightMotor1.getSelectedSensorPosition()),
            pose
                               );
    //leftPositonMeters and rightPositionMeters posibly should not be 0. Not sure.
    }

    public double getPitch() {
        return gyro.getPitch();
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

    public void waitForHeadingReliable() {

        timer.start();

        while (gyro.isCalibrating()) {

            if (timer.hasElapsed(Constants.Drivebase.WAIT_TIME_FOR_GYRO_CALIBRATION)) {

                System.out.println("gyro took too long to calibrate");
                System.out.println("heading reliability is " + isHeadingReliable);

                return;
            }
        }


        isHeadingReliable = true;

        System.out.println("gyro finished calibrating");
        System.out.println("heading reliability is " + isHeadingReliable);

    }

    public void setGyroStatus(boolean status) {
        isHeadingReliable = status;
    }

    public int getFrontRangeSensor() {
        return frontRangeSensorValue.getValue();
    }

    public int getBackRangeSensor() {
        return backRangeSensorValue.getValue();
    }

    public int getDirectionRangeSensor() {
        if (driveDirection == DriveDirection.FORWARD) return getFrontRangeSensor();
        if (driveDirection == DriveDirection.BACKWARD) return getBackRangeSensor();
        return 0;
    }



    public void setBackRangeSensor(boolean value) {
        backRangeSensorTrigger.set(value);
    }

    public void setFrontRangeSensor(boolean value) {
        frontRangeSensorTrigger.set(value);
    }

    public static Drivebase GetDrivebase() {

        if (OGDrivebase == null) {
            OGDrivebase = new Drivebase();
        }

        return OGDrivebase;

    }

    public void resetGyro()
    {
        gyro.reset();
    }
    public void resetGyroTo(double angle){
        gyro.reset();
        gyro.setAngleAdjustment(angle);
    }

    private double caculateDistance(Pose2d apriltag, Pose2d robot) {
        double YDistance = apriltag.getY() - robot.getY();
        double XDistance = apriltag.getX() - robot.getX();
        double overallDistance = Math.pow(Math.pow(YDistance, 2) + Math.pow(XDistance, 2), 0.5);
        return overallDistance;
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        //updateOdometry();//update odometry is just backup
        poseInitilized = false;
        poseEstimator.update(gyro.getRotation2d(),
                ticksToMeters((int) leftMotor1.getSelectedSensorPosition()),
                ticksToMeters((int) rightMotor1.getSelectedSensorPosition()));

        field.setRobotPose(poseEstimator.getEstimatedPosition());
        SmartDashboard.putData("field",field);
        double distToAprilTags;
        long aprilTagID = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tid").getInteger(-1);
        double ta = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("ta").getDouble(0.5);
        System.out.println("Apriltag " + aprilTagID);
        if(aprilTagID != -1 && ta > Constants.Drivebase.SMALLEST_APRIL_TAG_SIZE)
        {
            Double[] poseInfo = NetworkTableInstance.getDefault().getTable("limelight-front").
                    getEntry("botpose_wpired").getDoubleArray(new Double[]{0.0,0.0,0.0,0.0,0.0,0.0,0.0});
            Pose2d aprilTagPose = new Pose2d(poseInfo[0],poseInfo[1],new Rotation2d(Units.degreesToRadians(poseInfo[5])));
            System.out.println("the april tag pose is " + aprilTagPose.getY() + " " + aprilTagPose.getX());
            Pose2d robotPose = poseEstimator.getEstimatedPosition();
            System.out.println("the poseEstimator is " + robotPose.getY() + " " + robotPose.getX());
            double distanceFromAprilTag = caculateDistance(aprilTagPose,robotPose);
            Matrix<N3,N1> standardDeviationMatrix = new Matrix<N3,N1>(Nat.N3(), Nat.N1());
            standardDeviationMatrix.set(0,0, distanceFromAprilTag * Constants.Drivebase.VISUAL_ODOMETRY_STD_DEV_SCALE);
            standardDeviationMatrix.set(1,0, distanceFromAprilTag * Constants.Drivebase.VISUAL_ODOMETRY_STD_DEV_SCALE);
            if(poseInitilized == true)
            {
                standardDeviationMatrix.set(2,0, 500000000.0);
            }
            else
            {
                standardDeviationMatrix.set(0,0,.5);
                standardDeviationMatrix.set(1,0,.5);
                standardDeviationMatrix.set(2,0,.5);
            }
            double latency = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tl").getDouble(0.0)/1000;
            poseEstimator.addVisionMeasurement(aprilTagPose, Timer.getFPGATimestamp() - latency,standardDeviationMatrix);
            SmartDashboard.putNumber("aprilTagX", aprilTagPose.getX());
            SmartDashboard.putNumber("aprilTagY", aprilTagPose.getY());
            SmartDashboard.putNumber("aprilTagRotation", aprilTagPose.getRotation().getDegrees());
            SmartDashboard.putNumber("robotX", robotPose.getX());
            SmartDashboard.putNumber("robotY", robotPose.getY());
            SmartDashboard.putNumber("robotRotation", robotPose.getRotation().getDegrees());
            if(poseInitilized == false)
            {
            seeTag++;
            }
            if(seeTag >= 100 && !poseInitilized)
            {
                rightMotor1.setNeutralMode(NeutralMode.Coast);
                rightMotor2.setNeutralMode(NeutralMode.Coast);
                leftMotor1.setNeutralMode(NeutralMode.Coast);
                leftMotor2.setNeutralMode(NeutralMode.Coast);
                armInstance.SetLightMode(Constants.Lights.RED_WITH_WHITE);
                poseInitilized = true;
            }
        }
        Pose2d estimatedPos = poseEstimator.getEstimatedPosition();

        Logger.getInstance().recordOutput("DrivebaseEstimatedPose", estimatedPos);
        field.setRobotPose(estimatedPos);
        SmartDashboard.putData("field", field);

//TODO: if we have an april tag in view, call addVisionMeasurement();

    }

    public void setRightMeters(double meters){
        rightMotor1.set(ControlMode.Velocity,meters);
    }
    public void setLeftMeters(double meters){
        leftMotor1.set(ControlMode.Velocity,meters);//erm actualyy ticks should be fixed.
    }
    double ticksToMeters(double ticks){return ticksToFeet(ticks)/Constants.Drivebase.FEET_PER_METER;}

    double ticksToFeet(double ticks){

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
