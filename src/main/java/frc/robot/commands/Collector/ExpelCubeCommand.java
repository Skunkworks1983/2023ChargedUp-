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
