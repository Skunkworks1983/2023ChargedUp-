package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class RotateDegrees extends CommandBase {
    private final Arm arm;

    private double rotateTo;
    private final boolean ignore;
    private double startAngle;

    private boolean lastAngle;

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

        if (ignore) {
            rotateTo = startAngle + rotateTo;
        }

        double kF = (Constants.Arm.KF / (rotateTo / arm.encoderToAngleFactor)) * 1024;

        System.out.println("kF: " + kF);

        arm.configArmKF(kF);

        arm.setShoulderAnglePosition(rotateTo);

        System.out.println("Going to " + rotateTo);
    }

    @Override
    public void execute() {
        System.out.println("motor output: " + arm.getCurrentOutput());

        double angle = arm.getShoulderAngle();

        if (angle >= Constants.Arm.SWAP_ANGLE + Constants.Arm.SWAP_ANGLE_ADDITION) {
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
