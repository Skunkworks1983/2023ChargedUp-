package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;


public class ExpelCubeCommand extends CommandBase {
    private Collector collectorInstance;
    private Arm armInstance;

    public ExpelCubeCommand() {
        armInstance = Arm.getInstance();
        collectorInstance = Collector.getInstance();
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(collectorInstance);
    }

    @Override
    public void initialize() {
        System.out.println("Expel Cube Initialized");

        if(armInstance.getShoulderAngle() < 0) {
            collectorInstance.Setspeed(-Constants.Collector.EXPEL_MOTOR_SPEED);
        }
        else {
            collectorInstance.Setspeed(Constants.Collector.EXPEL_MOTOR_SPEED);
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
        collectorInstance.Setspeed(0);
        if(interrupted)
        {
            System.out.println("Expel Cube Command Ended, interrupted");
        }
        else
        {
            System.out.println("Expel Cube Command Ended");
        }
    }
}
