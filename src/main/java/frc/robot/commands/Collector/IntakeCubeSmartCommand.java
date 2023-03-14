package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;


public class IntakeCubeSmartCommand extends CommandBase {
    private Collector collectorInstance;
    private Arm armInstance;
    private int ticksElapsed;

    public IntakeCubeSmartCommand() {
        armInstance = Arm.getInstance();
        collectorInstance = Collector.getInstance();
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(collectorInstance);
        ticksElapsed = 0;
    }

    @Override
    public void initialize() {
        System.out.println("intake cube initialize");
    }

    @Override
    public void execute() {
        if (armInstance.getShoulderAngle() < 0) {

            if (collectorInstance.isIntaking()) {

                collectorInstance.Setspeed(Constants.Collector.INTAKE_MOTOR_SPEED_SLOW);

            } else {

                collectorInstance.Setspeed(Constants.Collector.INTAKE_MOTOR_SPEED);
            }

        } else {

            if (collectorInstance.isIntaking()) {

                collectorInstance.Setspeed(-Constants.Collector.INTAKE_MOTOR_SPEED_SLOW);

            } else {

                collectorInstance.Setspeed(-Constants.Collector.INTAKE_MOTOR_SPEED);


            }
        }
        ticksElapsed++;
    }

    @Override
    public boolean isFinished() {
        if(ticksElapsed >= Constants.Collector.CUBE_COLLECTED_MINIMUM_TICKS) {
            return collectorInstance.isHoldingCube() && ticksElapsed >= Constants.Collector.TICKS_BEFORE_FINISHED;
        }
        return false;
    }


    @Override
    public void end(boolean interrupted) {
        collectorInstance.Setspeed(0);
        if (interrupted) {
            System.out.println("Intake Cube Smart Command Ended, interrupted");
        } else {
            System.out.println("Intake Cube Smart Command Ended");
        }
    }
}
