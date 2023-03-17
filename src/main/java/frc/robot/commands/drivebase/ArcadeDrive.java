package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.services.Oi;
import frc.robot.subsystems.Drivebase;

public class ArcadeDrive extends CommandBase {
    private final Drivebase drivebase;
    private final Oi oi;

    private double targetHeading;

    private PIDController pidController = new PIDController(Constants.Drivebase.ARCADE_DRIVE_KP, 0, Constants.Drivebase.ARCADE_DRIVE_KD);


    public ArcadeDrive(Drivebase drivebase, Oi oi) {

        this.drivebase = drivebase;
        this.oi = oi;

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        targetHeading = drivebase.getHeading();
    }

    @Override
    public void execute() {
        double leftX = oi.getLeftX();
        double rightY = -oi.getRightY();

        double oldX = leftX;
        double oldY = rightY;

        leftX = Math.pow(leftX, 2) * (oldX < 0 ? -1 : 1);
        rightY = Math.pow(rightY, 2) * (oldY < 0 ? -1 : 1);

        double heading = drivebase.getHeading();

        double turnThrottle;

        if (!Double.isNaN(heading) && Math.abs(leftX) > Constants.Drivebase.ARCADE_DRIVE_LEFTX_DEADBAND) {
            targetHeading = targetHeading + ((Constants.Drivebase.ARCADE_DRIVE_MAX_DEGREES_PER_SECOND /
                    Constants.Drivebase.EXECUTES_PER_SECOND) * leftX);
            turnThrottle = pidController.calculate(heading, targetHeading);
        } else {
            turnThrottle = leftX;
        }

        SmartDashboard.putNumber("arcade drive turn error", pidController.getPositionError());
        SmartDashboard.putNumber("arcade drive turn joystick value", leftX);
        SmartDashboard.putNumber("arcade drive throttle joystick value", rightY);
        SmartDashboard.putNumber("arcade drive turn throttle", turnThrottle);

        double leftSpeed = rightY + turnThrottle;
        double rightSpeed = rightY - turnThrottle;

        leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
        rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);

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