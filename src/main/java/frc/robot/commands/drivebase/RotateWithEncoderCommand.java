package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;


public class RotateWithEncoderCommand extends CommandBase {

    private final Drivebase drivebase = Drivebase.GetDrivebase();
    private double targetDegree;

    private double startTicks;

    private double rotateDistance;
    private final PIDController pidController = new PIDController(0.00005, 0.00006, 0);

    public RotateWithEncoderCommand(double targetDegree) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.targetDegree = targetDegree * Math.PI / 180;

//        rotateDistance = ((Constants.Falcon500.TICKS_PER_REV * Constants.Drivebase.GEAR_RATIO) *
//                (Constants.Drivebase.DISTANCE_BETWEEN_WHEELS / 12) * this.targetDegree) /
//                (2 * Constants.Drivebase.WHEEL_DIAMETER * Math.PI);

        rotateDistance = ((Constants.Wobbles.TICKS_PER_MOTOR_REV * Constants.Wobbles.GEAR_RATIO) *
                (Constants.Wobbles.DISTANCE_BETWEEN_WHEELS / 12) * this.targetDegree) /
                (2 * Constants.Wobbles.WHEEL_DIAMETER * Math.PI);

        System.out.println("rotateDistance: " + rotateDistance);
        System.out.println("targetDegree: " + this.targetDegree);
    }

    @Override
    public void initialize() {
        startTicks = drivebase.getTicksLeft();

        System.out.println("startTicks: " + startTicks);

        SmartDashboard.putNumber("start ticks", startTicks);

        pidController.setSetpoint(startTicks - rotateDistance);
    }

    @Override
    public void execute() {
        double ticks = drivebase.getTicksLeft();

        double speed = pidController.calculate(ticks);

        speed = MathUtil.clamp(speed, -0.1, 0.1);

        System.out.println("speed: " + speed + " error: " + pidController.getPositionError());

        SmartDashboard.putNumber("rwe error", pidController.getPositionError());

        drivebase.runMotor(speed, -speed);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("rotate finished");
        drivebase.runMotor(0, 0);
    }
}
