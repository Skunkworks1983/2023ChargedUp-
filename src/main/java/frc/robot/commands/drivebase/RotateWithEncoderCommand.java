package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;


public class RotateWithEncoderCommand extends CommandBase {

    private final Drivebase drivebase = Drivebase.GetDrivebase();
    private double targetDegree;
    private double rotateDistance;

    private double leftSetpoint;
    private double rightSetpoint;

    private double leftStartTicks;
    private double rightStartTicks;

    public RotateWithEncoderCommand(double targetDegree) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.targetDegree = (targetDegree * Constants.Drivebase.ENCODER_ROTATE_INPUT_MULTIPLIER) * Math.PI / 180;

        rotateDistance = ((Constants.Falcon500.TICKS_PER_REV * Constants.Drivebase.GEAR_RATIO) *
                (Constants.Drivebase.DISTANCE_BETWEEN_WHEELS / 12) * this.targetDegree) /
                (2 * Constants.Drivebase.WHEEL_DIAMETER * Math.PI);

//        rotateDistance = ((Constants.Wobbles.TICKS_PER_MOTOR_REV * Constants.Wobbles.GEAR_RATIO) *
//                (Constants.Wobbles.DISTANCE_BETWEEN_WHEELS / 12.0) * this.targetDegree) /
//                (2 * Constants.Wobbles.WHEEL_DIAMETER * Math.PI);

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        leftStartTicks = drivebase.getTicksLeft();
        rightStartTicks = drivebase.getTicksRight();

        leftSetpoint = leftStartTicks - rotateDistance;
        rightSetpoint = rightStartTicks + rotateDistance;

        drivebase.setPosition(leftSetpoint, rightSetpoint);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return Math.abs(drivebase.getTicksLeft() - leftSetpoint) <= Constants.Drivebase.ROTATE_TOLERANCE &&
                Math.abs(drivebase.getTicksRight() - rightSetpoint) <= Constants.Drivebase.ROTATE_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("rotate finished");
        drivebase.runMotor(0, 0);
    }
}
