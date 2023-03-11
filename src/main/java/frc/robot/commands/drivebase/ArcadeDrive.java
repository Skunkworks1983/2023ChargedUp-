package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.services.Oi;
import frc.robot.subsystems.Drivebase;

import static java.lang.Double.NaN;


public class ArcadeDrive extends CommandBase {
    private final Drivebase drivebase;
    private final Oi oi;

    private double targetHeading;

    private PIDController pidController = new PIDController(Constants.Drivebase.ARCADE_DRIVE_KP, 0, Constants.Drivebase.ARCADE_DRIVE_KD);

    public ArcadeDrive(Drivebase drivebase, Oi oi) {
        this.drivebase = drivebase;
        this.oi = oi;
    }

    @Override
    public void initialize() {

        targetHeading = drivebase.getHeading();
    }

    @Override
    public void execute() {
        double leftX = oi.getLeftX();
        double rightY = oi.getRightY();

        double oldX = leftX;
        double oldY = rightY;

        leftX = Math.pow(leftX, 2) * (oldX < 0 ? -1 : 1);
        rightY = Math.pow(rightY, 2) * (oldY < 0 ? -1 : 1);

        double heading = drivebase.getHeading();
        double turnThrottle = 0;

        if (drivebase.getHeading() == NaN && Math.abs(leftX) > 0.01) {
            turnThrottle = leftX;
        }
        else if (Math.abs(leftX) > 0.01) {
            targetHeading = targetHeading - ((Constants.Drivebase.ARCADE_DRIVE_MAX_DEGREES_PER_SECOND/
                    Constants.Drivebase.EXECUTES_PER_SECOND)*leftX) ;

            turnThrottle = pidController.calculate(heading, targetHeading);
        }

//      //  System.out.println("error: " + pidController.getPositionError());
//       // System.out.println("heading: " + heading);
//        System.out.println("target heading: " + targetHeading);
//        System.out.println("turn throttle: " + turnThrottle);

//        if (Math.abs(leftX) > 0.01) {
//            turnThrottle = leftX;
//            targetHeading = drivebase.getHeading();
//        }
        //System.out.printf("Turn Speed: %f%n", turnThrottle);

        double leftSpeed = rightY + turnThrottle;
        double rightSpeed = rightY - turnThrottle;

//        System.out.printf("Left: %f | Right: %f%n", leftSpeed, rightSpeed);
//
//        System.out.printf("Target Heading: %f | Current Heading: %f%n", targetHeading, heading);

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