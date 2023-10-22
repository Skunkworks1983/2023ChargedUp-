package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.services.Oi;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.LimeLight;


public class ArcadeDrive extends CommandBase {
    private final Drivebase drivebase;
    private final Oi oi;
    private final LimeLight limeLight;

    private double targetHeading;

    private PIDController drivePidController = new PIDController(Constants.Drivebase.ARCADE_DRIVE_KP, 0, Constants.Drivebase.ARCADE_DRIVE_KD);
    private PIDController limeLightPidController = new PIDController(Constants.Drivebase.DRIVE_TO_CONE_KP, 0, 0);

    public ArcadeDrive(Drivebase drivebase, Oi oi, LimeLight limeLight) {

        this.drivebase = drivebase;
        this.oi = oi;
        this.limeLight = limeLight;

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        targetHeading = drivebase.getHeading();
        System.out.println("Arcadedrive beginning");

        System.out.println("turning to game piece init!!!");
    }

    @Override
    public void execute() {

        double leftX = oi.getLeftX();
        double rightY = -oi.getRightY();

        double oldX = leftX;
        double oldY = rightY;

        leftX = (Math.pow(Math.abs(leftX), 2.2)) * (oldX < 0 ? -1 : 1) * .75;
        rightY = (Math.pow(Math.abs(rightY), 2)) * (oldY < 0 ? -1 : 1) * .80;

        double heading = drivebase.getHeading();

        double turnThrottle;

       /* if (!Double.isNaN(heading) && Math.abs(leftX) > Constants.Drivebase.ARCADE_DRIVE_LEFTX_DEADBAND) {
            targetHeading = targetHeading + ((Constants.Drivebase.ARCADE_DRIVE_MAX_DEGREES_PER_SECOND /
                    Constants.Drivebase.EXECUTES_PER_SECOND) * leftX);
            turnThrottle = pidController.calculate(heading, targetHeading);
        } else {*/

        //}

        if (oi.isCenterOnPiece()) {

            turnThrottle = limeLight.getLimeX();

        } else {

            turnThrottle = leftX * Constants.Drivebase.TURN_THROTTLE_MULTIPLIER;
        }

//        SmartDashboard.putNumber("arcade drive lime light turn error", limeLightPidController.getPositionError());
//        SmartDashboard.putNumber("arcade drive turn joystick value", leftX);
//        SmartDashboard.putNumber("arcade drive throttle joystick value", rightY);
//        SmartDashboard.putNumber("arcade drive turn throttle", turnThrottle);

        double leftSpeed = rightY + turnThrottle;
        double rightSpeed = rightY - turnThrottle;

        leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
        rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);

        if (Oi.GetInstance().isSlowMode()) {

            drivebase.runMotor(leftSpeed * Constants.Drivebase.SLOW_MODE_RATIO,
                    rightSpeed * Constants.Drivebase.SLOW_MODE_RATIO);

        } else {

            drivebase.runMotor(leftSpeed, rightSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Arcade drive end Interrupted " + interrupted);
    }
}