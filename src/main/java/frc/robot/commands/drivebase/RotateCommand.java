package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class RotateCommand extends CommandBase {
    private final Drivebase drivebase;
    private double degree;
    private double startDegree;
    private double finishDegree;
    private int onTargetCount;
    private boolean absolute;
    private PIDController pidController = new PIDController(Constants.Drivebase.ANGLE_KP, 0.0032, Constants.Drivebase.ANGLE_KD, Constants.Drivebase.DRIVEBASE_KF);

    // Encoder rotate stuff
    private double leftSetpoint;
    private double rightSetpoint;

    private double leftStartTicks;
    private double rightStartTicks;

    private double rotateDistance;

    private double lastGyroHeading;

    private boolean useGyroRotate = true;

    public RotateCommand(Drivebase drivebase, double degree) {
        this(drivebase, degree, false);
    }

    // TODO:
    // Remove absolute

    public RotateCommand(Drivebase drivebase, double degree, boolean absolute) {
        addRequirements(drivebase);

        this.drivebase = drivebase;
        this.degree = degree;
        this.absolute = absolute;

        pidController.setIntegratorRange(-0.2, 0.2);
    }

    @Override
    public void initialize() {
        if (Double.isNaN(drivebase.getHeading())) {
            System.out.println("gyro crashed in init, switching to encoder rotate");

            useGyroRotate = false;
        }

        if (useGyroRotate) {
            SmartDashboard.putString("gyro rotate", ":)");
        } else {
            SmartDashboard.putString("gyro rotate", ":(");
        }

        degree = (degree * Constants.Drivebase.ENCODER_ROTATE_INPUT_MULTIPLIER) * Math.PI / 180;

        // !! Comp Bot !!
        rotateDistance = ((Constants.Falcon500.TICKS_PER_REV * Constants.Drivebase.GEAR_RATIO) *
                (Constants.Drivebase.DISTANCE_BETWEEN_WHEELS / 12) * degree) /
                (2 * Constants.Drivebase.WHEEL_DIAMETER * Math.PI);

        // !! Wobbles !!
//            rotateDistance = ((Constants.Wobbles.TICKS_PER_MOTOR_REV * Constants.Wobbles.GEAR_RATIO) *
//                    (Constants.Wobbles.DISTANCE_BETWEEN_WHEELS / 12) * degree) /
//                    (2 * Constants.Wobbles.WHEEL_DIAMETER * Math.PI);

        leftStartTicks = drivebase.getTicksLeft();
        rightStartTicks = drivebase.getTicksRight();

        leftSetpoint = leftStartTicks - rotateDistance;
        rightSetpoint = rightStartTicks + rotateDistance;

        if (useGyroRotate) {
            startDegree = drivebase.getHeading();
            if (absolute) {
                finishDegree = degree;
            } else {
                finishDegree = startDegree + degree;
            }
        } else {
            drivebase.setPosition(leftSetpoint, rightSetpoint);
        }


        System.out.println("starting RotateCommand");
    }

    @Override
    public void execute() {
        // TODO: remove debug
        if (useGyroRotate && Math.abs(finishDegree - lastGyroHeading) <= 90) {
            System.out.println("failed gyro with debug");
            useGyroRotate = false;
        }
        //TODO: remove debug v2
        //not to be confused with the code below
        if (useGyroRotate) {
            if (drivebase.gyroCalibrating() || !drivebase.gyroConnected()) {
                System.out.println("gyro crashed during rotate, calibrating or not connected");
                useGyroRotate = false;
                drivebase.setPosition(leftSetpoint, rightSetpoint);
            }
            if (Math.abs(drivebase.getHeading() - lastGyroHeading) >= Constants.Drivebase.HEADING_TOO_BIG) {
                System.out.println("gyro crashed during rotate, heading too large");
                useGyroRotate = false;
                drivebase.setPosition(leftSetpoint, rightSetpoint);
            }
        }

        if (useGyroRotate) {
            double heading = drivebase.getHeading();

            lastGyroHeading = heading;

            double speed = pidController.calculate(heading, finishDegree);
            if (speed > 0.5) {
                speed = 0.5;
            } else if (speed < -0.5) {
                speed = -0.5;
            }
            drivebase.runMotor(speed, -speed);
            //System.out.println(" speed: " + speed);
        }
    }

    @Override
    public boolean isFinished() {
        if (useGyroRotate) {
            if (Math.abs(drivebase.getHeading() - finishDegree) < Constants.Drivebase.THRESHOLD_ROTATE) {
                onTargetCount++;
            } else {
                onTargetCount = 0;
            }
            return onTargetCount >= 3;
        } else {
//            SmartDashboard.putNumber("left ticks minus setpoint", Math.abs(drivebase.getTicksLeft() - leftSetpoint));
//            SmartDashboard.putNumber("right ticks minus setpoint", Math.abs(drivebase.getTicksRight() - rightSetpoint));
            return Math.abs(drivebase.getTicksLeft() - leftSetpoint) <= Constants.Drivebase.ROTATE_TOLERANCE &&
                    Math.abs(drivebase.getTicksRight() - rightSetpoint) <= Constants.Drivebase.ROTATE_TOLERANCE;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.runMotor(0, 0);
        System.out.println("ending RotateCommand");
    }
}