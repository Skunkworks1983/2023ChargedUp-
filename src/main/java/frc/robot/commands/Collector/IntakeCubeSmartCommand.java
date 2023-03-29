package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivebase;


public class IntakeCubeSmartCommand extends CommandBase
{
    private final Collector collectorInstance;
    private final Arm armInstance;
    private int ticksElapsed;
    private final Drivebase drivebaseInstance = Drivebase.GetDrivebase();

    public IntakeCubeSmartCommand()
    {

        armInstance = Arm.getInstance();
        collectorInstance = Collector.getInstance();
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(collectorInstance);

    }

    @Override
    public void initialize()
    {
        ticksElapsed = 0;
        System.out.println("intake cube initialize");
    }

    @Override
    public void execute()
    {
        double Speed = Constants.Collector.INTAKE_MOTOR_SPEED * drivebaseInstance.currentMotorSpeed
                + Constants.Collector.INTAKE_MOTOR_SPEED * Constants.Collector.SCALE_BY_DRIVEBASE * (
                drivebaseInstance.currentMotorSpeed - 1);
        //Collector Velocity=M(Vd-0) + MK(Vd-1) this is the formula
        if(collectorInstance.isIntakingCube())
        {
            collectorInstance.SetSpeed(armInstance.getCurrentPose().CubeIntake() * Constants.Collector.INTAKE_MOTOR_SPEED_SLOW);
        }
        else if(collectorInstance.isHoldingCube())
        {
            collectorInstance.SetSpeed(armInstance.getCurrentPose().CubeIntake() * Constants.Collector.INTAKE_MOTOR_SPEED_VERY_SLOW);
        }
        else
        {
            collectorInstance.SetSpeed(armInstance.getCurrentPose().CubeIntake() * Speed);
        }
        ticksElapsed++;
    }

    @Override
    public boolean isFinished()
    {
        return collectorInstance.isHoldingCube() && ticksElapsed >= Constants.Collector.TICKS_BEFORE_FINISHED;
    }


    @Override
    public void end(boolean interrupted)
    {
        collectorInstance.SetSpeed(0);
        if(interrupted)
        {
            System.out.println("Intake Cube Smart Command Ended, interrupted");
        }
        else
        {
            System.out.println("Intake Cube Smart Command Ended");
        }
    }
}
