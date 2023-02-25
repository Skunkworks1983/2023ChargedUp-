package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Collector;


public class CollectorIntakeAutoCommand extends CommandBase {
    public Collector collectorInstance;

    public CollectorIntakeAutoCommand() {
        collectorInstance = Collector.getInstance();
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
        collectorInstance.Setspeed(Constants.Collector.INTAKE_MOTOR_SPEED_2022);

    }

    @Override
    public void execute() {
    collectorInstance.CubebreakPrint();

    }

    @Override
    public boolean isFinished() {
        //return collectorInstance.cubeCollected();
        return collectorInstance.cubeCollected();
    }

    @Override
    public void end(boolean interrupted) {
        collectorInstance.Setspeed(0);
    }
}
