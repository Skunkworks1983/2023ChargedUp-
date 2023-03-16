package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;


public class ExpelConeCommand extends CommandBase {
    private Collector collectorInstance;
    private Arm armInstance;

    public ExpelConeCommand() {
        armInstance = Arm.getInstance();
        collectorInstance = Collector.getInstance();
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(collectorInstance);
    }

    @Override
    public void initialize() {
        System.out.println("Expel Cone Initialized");

        if(armInstance.getShoulderAngle() < 0) {
            collectorInstance.SetspeedVelocity(Constants.Collector.EXPEL_MOTOR_SPEED);
        }
        else {
            collectorInstance.SetspeedVelocity(-Constants.Collector.EXPEL_MOTOR_SPEED);
        }



    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        collectorInstance.SetspeedVelocity(0);
        if(interrupted)
        {
            System.out.println("Expel Cone Command ended, interrupted");
        }
        else
        {
            System.out.println("Expel Cone Command ended");
        }
    }
}
