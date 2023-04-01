package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.LimeLight;


public class DriveToGamePieceCommand extends CommandBase {

    private final Drivebase drivebase;
    private final LimeLight limeLight;
    PIDController pidController = new PIDController
            (Constants.Drivebase.DRIVE_TO_CONE_KP, 0, Constants.Drivebase.DRIVE_TO_CONE_KD);

    private double speed;
    private double areaThreshold;


    public DriveToGamePieceCommand(double speed, double areaThreshold) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.speed = speed;
        this.areaThreshold = areaThreshold;
        drivebase = Drivebase.GetDrivebase();
        limeLight = LimeLight.getInstance();
        addRequirements(drivebase);
        pidController.setSetpoint(0);
    }

    @Override
    public void initialize() {

//        double limeLightX = tx.getDouble(0.0);
//        double limeLightY = ty.getDouble(0.0);
//        double limeLightA = ta.getDouble(0.0);

//        SmartDashboard.putNumber("LimelightX", limeLightX);
//        SmartDashboard.putNumber("LimelightY", limeLightY);
//        SmartDashboard.putNumber("LimelightArea", limeLightA);
        limeLight.setEnable(true);

        System.out.println("initializing drive to game piece command!!!");
    }

    @Override
    public void execute() {

        double driveThrottle = speed;
        double turnThrottle = pidController.calculate(limeLight.getLimeX());

        double leftSpeed = driveThrottle - turnThrottle; //calculates leftSpeed and rightSpeed
        double rightSpeed = driveThrottle + turnThrottle;

        leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
        rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);

        drivebase.runMotor(leftSpeed, rightSpeed);

        System.out.println("turnThrottle " + turnThrottle);
        System.out.println("leftSpeed " + leftSpeed + "rightSpeed " + rightSpeed);
        System.out.println("TX: " + limeLight.getLimeX() + " TY: " + limeLight.getLimeY());
    }

    @Override
    public boolean isFinished() {

        return (limeLight.getLimeA() >= areaThreshold);
    }


    @Override
    public void end(boolean interrupted) {

        drivebase.runMotor(0, 0);

        drivebase.SetBrakeMode(true);
    }
}
