package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class ShoulderRotateDegrees extends CommandBase {
    private final Arm arm;
    private final double offsetDegrees;
    private double rotateTo;
    private final boolean ignore;
    private double startAngle;

    private boolean lastAngle;

    public ShoulderRotateDegrees(Arm arm, double offsetDegrees, boolean ignore) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.arm = arm;
        this.offsetDegrees = offsetDegrees;
        this.ignore = ignore;

        System.out.println("rotate degrees constructor");
    }

    @Override
    public void initialize() {
        System.out.println("rotate degrees init");

        startAngle = arm.getShoulderAngle();

        rotateTo = offsetDegrees;

        System.out.println("Starting at " + startAngle);

        if (ignore) {
            rotateTo = startAngle + offsetDegrees;
        }

        arm.setShoulderAnglePosition(rotateTo);

        System.out.println("Going to " + rotateTo);
    }

    @Override
    public void execute() {
        //System.out.println("motor output: " + arm.getCurrentOutput());

        double angle = arm.getShoulderAngle();

        if (angle >= Constants.Arm.SHOULDER_SWAP_ANGLE + Constants.Arm.SHOULDER_SWAP_ANGLE_ADDITION) {
            if (!lastAngle) {
                lastAngle = true;
                /*
                todo

                add arm code
                */
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(arm.ShoulderMotor.getClosedLoopError() * Constants.Arm.SHOULDER_TICKS_TO_DEGREES) < Constants.Arm.SHOULDER_TOLERANCE) {
            System.out.println("Ended");
            System.out.println("end error: " + arm.ShoulderMotor.getClosedLoopError() * Constants.Arm.SHOULDER_TICKS_TO_DEGREES);
            return false; //todo
        } else {
            return false;
        }
        //return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("actual end");
    }
}
