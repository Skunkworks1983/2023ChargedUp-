package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.services.Oi;
import frc.robot.subsystems.Drivebase;

import java.util.ArrayList;

public class ArcadeDrive extends CommandBase {
    private final Drivebase drivebase;
    private final Oi oi;

    private double targetHeading;

    private PIDController pidController = new PIDController(Constants.Drivebase.ARCADE_DRIVE_KP, 0, Constants.Drivebase.ARCADE_DRIVE_KD);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    ArrayList<Double> listX = new ArrayList<>();

    public ArcadeDrive(Drivebase drivebase, Oi oi) {

        this.drivebase = drivebase;
        this.oi = oi;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        targetHeading = drivebase.getHeading();
        System.out.println("Arcadedrive beginning");

        System.out.println("turning to game piece init!!!");

        System.out.println("initial limelight X " + tx.getDouble(0.0));
    }

    @Override
    public void execute() {

        double limeX = tx.getDouble(0.0); //sets limeX to current x value
        listX.add(limeX);
        if (listX.size() > Constants.Drivebase.ROLLING_AVERAGE_LENGTH) {
            listX.remove(0);
        }

        double sumX = 0;

        for (double listItem : listX) {
            sumX = sumX + listItem;
        }

        double averageX = sumX / listX.size();


        double leftX = oi.getLeftX();
        double rightY = -oi.getRightY();

        double oldX = leftX;
        double oldY = rightY;

        leftX = (Math.pow(Math.abs(leftX), 2.2)) * (oldX < 0 ? -1 : 1);
        rightY = (Math.pow(Math.abs(rightY), 2)) * (oldY < 0 ? -1 : 1);

        double heading = drivebase.getHeading();

        double turnThrottle;

       /* if (!Double.isNaN(heading) && Math.abs(leftX) > Constants.Drivebase.ARCADE_DRIVE_LEFTX_DEADBAND) {
            targetHeading = targetHeading + ((Constants.Drivebase.ARCADE_DRIVE_MAX_DEGREES_PER_SECOND /
                    Constants.Drivebase.EXECUTES_PER_SECOND) * leftX);
            turnThrottle = pidController.calculate(heading, targetHeading);
        } else {*/

    //}

        if (oi.isCenterOnPiece()) {

            turnThrottle = pidController.calculate
                    (averageX, Constants.Drivebase.LIMELIGHT_CAMERA_PIXEL_WIDTH/2);

        } else {

            turnThrottle = leftX * Constants.Drivebase.TURN_THROTTLE_MULTIPLIER;
        }

        SmartDashboard.putNumber("arcade drive turn error", pidController.getPositionError());
        SmartDashboard.putNumber("arcade drive turn joystick value", leftX);
        SmartDashboard.putNumber("arcade drive throttle joystick value", rightY);
        SmartDashboard.putNumber("arcade drive turn throttle", turnThrottle);

        double leftSpeed = rightY + turnThrottle;
        double rightSpeed = rightY - turnThrottle;

        leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
        rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);

        if (oi.isSlowMode()) {

        drivebase.runMotor(leftSpeed*Constants.Drivebase.SLOW_MODE_RATIO,
                rightSpeed*Constants.Drivebase.SLOW_MODE_RATIO);

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
        System.out.println("Arcadedrive end Inturr " +interrupted );
    }
}