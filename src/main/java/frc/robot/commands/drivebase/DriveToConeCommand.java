package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;


public class DriveToConeCommand extends CommandBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    //NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    private final Drivebase drivebase;
    //double pixelError; //can be anywhere from -160 to 160

    ArrayList<Double> listA = new ArrayList<>();
    ArrayList<Double> listX = new ArrayList<>();

    PIDController pidController = new PIDController(Constants.Drivebase.DRIVE_TO_CONE_KP, 0, 0);

    public DriveToConeCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        drivebase = Drivebase.GetDrivebase();
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {

//        double limeLightX = tx.getDouble(0.0);
//        double limeLightY = ty.getDouble(0.0);
//        double limeLightA = ta.getDouble(0.0);

//        SmartDashboard.putNumber("LimelightX", limeLightX);
//        SmartDashboard.putNumber("LimelightY", limeLightY);
//        SmartDashboard.putNumber("LimelightArea", limeLightA);

    }

    @Override
    public void execute() {
        double limeA = ta.getDouble(0.0);
        listA.add(limeA);
        if (listA.size() > Constants.Drivebase.ROLLING_AVERAGE_LENGTH) {
            listA.remove(0);
        }

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

        double driveThrottle = Constants.Drivebase.BASE_DRIVE_TO_CONE_SPEED;
        double turnThrottle = pidController.calculate(0,
                averageX - (Constants.Drivebase.LIMELIGHT_CAMERA_PIXEL_WIDTH / 2));

        double leftSpeed = driveThrottle + turnThrottle; //calculates leftSpeed and rightSpeed
        double rightSpeed = driveThrottle - turnThrottle;

        leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
        rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);

        drivebase.runMotor(leftSpeed, rightSpeed);

        System.out.println("limeX " + limeX);
        System.out.println("limeA " + limeA);
        System.out.println("averageX " + averageX);
        System.out.println("turnThrottle " + turnThrottle);
        System.out.println("leftSpeed " + leftSpeed  + "rightSpeed " + rightSpeed);
    }

    @Override
    public boolean isFinished() {
        double sumA = 0;
        for (double listItem : listA) {
            sumA = sumA + listItem;
        }
        double averageA = sumA / listA.size();

        return (averageA > Constants.Drivebase.LIMELIGHT_MAX_CONE_AREA);
    }


    @Override
    public void end(boolean interrupted) {

        drivebase.runMotor(0, 0);

        drivebase.SetBrakeMode(true);
    }
}
