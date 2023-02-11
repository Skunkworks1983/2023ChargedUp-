package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RotateDegrees extends CommandBase {
    private final Arm arm;

    private double rotateTo;
    private final boolean ignore;
    private double startAngle;

    public RotateDegrees(Arm arm, double rotateTo, boolean ignore) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.arm = arm;
        this.rotateTo = rotateTo;
        this.ignore = ignore;
    }

    @Override
    public void initialize() {
        startAngle = arm.getShoulderAngle();

        System.out.println("Starting at " + startAngle);

        if (!ignore) {
            arm.setShoulderAnglePosition(rotateTo);
        } else {
            rotateTo = startAngle + rotateTo;
            arm.setShoulderAnglePosition(rotateTo);
        }

        System.out.println("Going to " + rotateTo);
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished() {
        System.out.println("current pos: " + Math.abs(arm.getShoulderAngle()));
        System.out.println(arm.getShoulderAngle() - rotateTo);
        if (Math.abs(arm.getShoulderAngle() - rotateTo) < 1) {
            System.out.println("Ended");
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted)
    {

    }
}
