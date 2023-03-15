package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.subsystems.Arm;

public class SetArmRaceCommandGroup extends ParallelRaceGroup
{

    public SetArmRaceCommandGroup(Arm.PoseType poseType, double shoulderPos, double wristPos, double timerSecond)
    {
        super((Command) new SetArmPositionCommand(poseType, shoulderPos,wristPos), new TimerCommand(timerSecond));
    }
}