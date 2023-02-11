package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ResetArm extends CommandBase {
    private final Arm arm;

    private double rotateTo;
    private double startAngle;

    public ResetArm(Arm arm, double rotateTo) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.arm = arm;
        this.rotateTo = rotateTo;
    }

    @Override
    public void initialize() {
        startAngle = arm.getShoulderAngle();

        System.out.println("Starting at " + startAngle);

        arm.setCollectorAnglePosition(rotateTo);

        System.out.println("Going to " + rotateTo);
    }

    @Override
    public void execute() {
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
    public void end(boolean interrupted) {

    }
}
