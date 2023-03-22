package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class SetArmRaceCommandGroup extends ParallelRaceGroup
{

    public SetArmRaceCommandGroup(Constants.ArmPose armPose, double timerSecond)
    {
        // TODO: cube vs cone?
        super((Command) new SetArmPositionCommand(armPose), new TimerCommand(timerSecond));
    }
}