package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;


public class IntakeConeSmartCommand extends CommandBase {
    private Collector collectorInstance;
    private Arm armInstance;
    private int countConeHeld;
    public IntakeConeSmartCommand() {
        armInstance = Arm.getInstance();
        collectorInstance = Collector.getInstance();
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(collectorInstance);
    }

    @Override
    public void initialize() {
        countConeHeld = 0;



        if(armInstance.getShoulderAngle() < 0) {
            collectorInstance.Setspeed(-Constants.Collector.INTAKE_MOTOR_SPEED);
        }
        else {
            collectorInstance.Setspeed(Constants.Collector.INTAKE_MOTOR_SPEED);
        }



    }

    @Override
    public void execute()
    {

        if(collectorInstance.isHoldingCone()) {
            countConeHeld++;
        }
        else{
            countConeHeld = 0;
        }
    }

    @Override
    public boolean isFinished() {
        System.out.println("the collector says its been holding cone " + countConeHeld);
        return countConeHeld >= Constants.Collector.CONE_COLLECTED_VALUE;

    }

    @Override
    public void end(boolean interrupted) {
        collectorInstance.Setspeed(0);
    }
}
