package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;


public class IntakeConeSmartCommand extends CommandBase {
    private Collector collectorInstance;
    private Arm armInstance;
    private int countConeHeld;
    private int ticksElapsed;

    public IntakeConeSmartCommand() {
        armInstance = Arm.getInstance();
        collectorInstance = Collector.getInstance();

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(collectorInstance);
    }

    @Override
    public void initialize() {
        System.out.println("Intake Cone Smart Command Initialize");
        countConeHeld = 0;
        ticksElapsed = 0;
    }


    @Override
    public void execute() {
      switch (armInstance.getCurrentPosition()) {
          case FLOOR:
          case FLOOR_NORMAL:
          case HIGH_CUBE:
              collectorInstance.Setspeed(-Constants.Collector.INTAKE_MOTOR_SPEED);
              break;
          case FLOOR_WEIRD:
          case SCORE_MID:
          case SUBSTATION:
              collectorInstance.Setspeed(Constants.Collector.INTAKE_MOTOR_SPEED);
              break;


           }
        if (collectorInstance.isHoldingCone()) {
            countConeHeld++;
        } else {
            countConeHeld = 0;
        }
        ticksElapsed++;
    }

    @Override
    public boolean isFinished() {
        return countConeHeld >= Constants.Collector.CONE_COLLECTED_VALUE && ticksElapsed >= Constants.Collector.TICKS_BEFORE_FINISHED;
    }

    @Override
    public void end(boolean interrupted) {
        collectorInstance.Setspeed(0);
        if (interrupted) {
            System.out.println("Intake Cone Smart Command Ended, interrupted");
        } else {
            System.out.println("Intake Cone Smart Command Ended");
        }
    }
}
