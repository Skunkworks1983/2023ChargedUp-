package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class WristRotateDegrees extends CommandBase {
    private final Arm arm;
    private final double offsetDegrees;
    private double rotateTo;
    private final boolean ignore;
    private double startAngle;

    private boolean lastAngle;

    public WristRotateDegrees(Arm arm, double offsetDegrees, boolean ignore) {
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

        startAngle = arm.getWristAngle();

        rotateTo = offsetDegrees;

        System.out.println("Starting at " + startAngle);

        if (ignore) {
            rotateTo = startAngle + offsetDegrees;
        }

        arm.setWristAnglePosition(rotateTo);

        System.out.println("Going to " + rotateTo);
    }

    @Override
    public void execute() {
        //System.out.println("motor output: " + arm.getCurrentOutput());
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(arm.WristMotor.getClosedLoopError() * Constants.Arm.WRIST_TICKS_TO_DEGREES) < Constants.Arm.WRIST_TOLERANCE) {
            System.out.println("Ended");
            System.out.println("end error: " + arm.WristMotor.getClosedLoopError() * Constants.Arm.WRIST_TICKS_TO_DEGREES);
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
