package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.subsystems.Arm;

public class SetArmRaceCommandGroup extends ParallelRaceGroup
{

    public SetArmRaceCommandGroup(Arm.ArmPosition armPosition, Arm.PostionPieceType postionPieceType, double timerSecond)
    {
        // TODO: cube vs cone?
        super((Command) new SetArmPositionCommand(armPosition, postionPieceType), new TimerCommand(timerSecond));
    }
}