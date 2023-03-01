package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.services.Oi;
import frc.robot.subsystems.multidrivebase.Drivebase;


public class ArcadeDrive extends CommandBase {
    private final Drivebase drivebase;
    private final Oi oi;

    private double targetHeading;

    private PIDController pidController = new PIDController(Constants.Drivebase.ARCADE_DRIVE_KP, 0, 0);

    public ArcadeDrive(Drivebase drivebase, Oi oi) {
        this.drivebase = drivebase;
        this.oi = oi;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        drivebase.resetGyro();
        drivebase.calibrateGyro();

        targetHeading = drivebase.getHeading();
    }

    @Override
    public void execute() {
        double leftX = oi.getLeftX();
        double rightY = oi.getRightY();

        double heading = drivebase.getHeading();

        double turnThrottle = pidController.calculate(heading, targetHeading);

        if (Math.abs(leftX) > 0.05) {
            turnThrottle = leftX;
            targetHeading = drivebase.getHeading();
        }

        //System.out.printf("Turn Speed: %f%n", turnThrottle);

        double leftSpeed = rightY + turnThrottle;
        double rightSpeed = rightY - turnThrottle;

        leftSpeed = MathUtil.clamp(leftSpeed, -0.7, 0.7);
        rightSpeed = MathUtil.clamp(rightSpeed, -0.7, 0.7);

        System.out.printf("Left: %f | Right: %f%n", leftSpeed, rightSpeed);

        System.out.printf("Target Heading: %f | Current Heading: %f%n", targetHeading, heading);

        drivebase.runMotor(leftSpeed, rightSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
