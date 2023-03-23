package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;


public class RotateWithEncoderCommand extends CommandBase {

    private final Drivebase drivebase;
    private double targetDegree;

    private double startTicks;

    private double rotateDistance;
    private PIDController pidController = new PIDController(Constants.Drivebase.ROTATE_KP, 0.0032, Constants.Drivebase.ANGLE_KD, Constants.Drivebase.DRIVEBASE_KF);

    public RotateWithEncoderCommand(Drivebase drivebase, double targetDegree) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.drivebase = drivebase;
        this.targetDegree = targetDegree * Math.PI / 180;
        addRequirements(drivebase);

        rotateDistance = (Constants.Falcon500.TICKS_PER_REV * Constants.Drivebase.DISTANCE_BETWEEN_WHEELS * this.targetDegree) /
                (2 * Constants.Drivebase.WHEEL_DIAMETER * Math.PI);

        System.out.println("rotateDistance: " + rotateDistance);
        System.out.println("targetDegree: " + this.targetDegree);
    }

    @Override
    public void initialize() {
        startTicks = drivebase.getTicksLeft();

        System.out.println("startTicks: " + startTicks);

        pidController.setSetpoint(startTicks + rotateDistance);
    }

    @Override
    public void execute() {
        double ticks = drivebase.getTicksLeft();

        double speed = pidController.calculate(ticks);

        drivebase.runMotor(-speed, speed);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.runMotor(0, 0);
    }
}
