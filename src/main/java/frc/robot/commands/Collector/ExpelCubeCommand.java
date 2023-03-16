package frc.robot.commands.Collector;

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

        if(armInstance.isArmForward()) {
            collectorInstance.SetSpeedVelocity(Constants.Collector.INTAKE_MOTOR_SPEED);
        }
        else {
            collectorInstance.SetSpeedVelocity(-Constants.Collector.INTAKE_MOTOR_SPEED);
        }



    }

    @Override
    public void execute()
    {
        if(armInstance.getShoulderAngle() -armInstance.getWristAngle() < 180 - Constants.Arm.WRIST_LIMIT_ANGLE) {
            collectorInstance.SetSpeedVelocity(Constants.Collector.INTAKE_MOTOR_SPEED);
        }
        else {
            collectorInstance.SetSpeedVelocity(-Constants.Collector.INTAKE_MOTOR_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        collectorInstance.SetSpeedVelocity(0);
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
