package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.LimeLight;


public class DriveToGamePieceCommand extends CommandBase {

    private final Drivebase drivebase;
    private final LimeLight limeLight;
    PIDController turnPidController = new PIDController
            (Constants.Drivebase.DRIVE_TO_CONE_KP, 0, Constants.Drivebase.DRIVE_TO_CONE_KD);

    PIDController drivePidController = new PIDController
            (Constants.Drivebase.SLOW_DOWN_TO_CONE_KP, 0, 0);



    public DriveToGamePieceCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        drivebase = Drivebase.GetDrivebase();
        limeLight = LimeLight.getInstance();
        addRequirements(drivebase);
        turnPidController.setSetpoint(0);
        drivePidController.setSetpoint(Constants.Drivebase.LIMELIGHT_MAX_CONE_AREA);
    }

    @Override
    public void initialize() {

//        double limeLightX = tx.getDouble(0.0);
//        double limeLightY = ty.getDouble(0.0);
//        double limeLightA = ta.getDouble(0.0);

//        SmartDashboard.putNumber("LimelightX", limeLightX);
//        SmartDashboard.putNumber("LimelightY", limeLightY);
//        SmartDashboard.putNumber("LimelightArea", limeLightA);

        System.out.println("initializing drive to game piece command!!!");
    }

    @Override
    public void execute() {

        double driveThrottle = Constants.Drivebase.BASE_DRIVE_TO_CONE_SPEED;
        double turnThrottle = turnPidController.calculate(limeLight.getLimeX());

        double leftSpeed = driveThrottle - turnThrottle; //calculates leftSpeed and rightSpeed
        double rightSpeed = driveThrottle + turnThrottle;

        leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
        rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);



        double limeA = limeLight.getLimeA();

        if (limeA > Constants.Drivebase.LIMELIGHT_SLOW_DOWN_AREA) {

            double ratio = drivePidController.calculate(limeA);

            drivebase.runMotor
                    (ratio*leftSpeed, ratio*rightSpeed);
        } else {

            drivebase.runMotor(leftSpeed, rightSpeed);
        }

        System.out.println("turnThrottle " + turnThrottle);
        System.out.println("leftSpeed " + leftSpeed  + "rightSpeed " + rightSpeed);
        System.out.println("TX: " + limeLight.getLimeX() + " TY: " + limeLight.getLimeY());
    }

    @Override
    public boolean isFinished() {

        return (limeLight.getLimeA() > Constants.Drivebase.LIMELIGHT_MAX_CONE_AREA);
    }


    @Override
    public void end(boolean interrupted) {

        drivebase.runMotor(0, 0);

        drivebase.SetBrakeMode(true);
    }
}
