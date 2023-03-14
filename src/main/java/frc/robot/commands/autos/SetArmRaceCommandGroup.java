package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.arm.SetArmPositionCommand;

public class SetArmRaceCommandGroup extends ParallelRaceGroup
{

    public SetArmRaceCommandGroup(double shoulderPos, double wristPos,double timerSecond)
    {
        super((Command) new SetArmPositionCommand(shoulderPos,wristPos), new TimerCommand(timerSecond));
    }
}