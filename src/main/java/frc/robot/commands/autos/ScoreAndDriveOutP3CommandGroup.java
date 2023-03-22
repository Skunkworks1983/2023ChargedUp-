package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;

public class ScoreAndDriveOutP3CommandGroup extends SequentialCommandGroup
{

    public ScoreAndDriveOutP3CommandGroup() {
        super(new SetArmRaceCommandGroup(Arm.ArmPosition.SCORE_MID, Arm.PostionPieceType.CONE, 1.75),
                new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2)),
                new SetArmRaceCommandGroup(Arm.ArmPosition.STOW, Arm.PostionPieceType.DOESNT_MATTER,.75),
                new DriveDistanceCommand(Drivebase.GetDrivebase(),-.5),new DriveDistanceCommand(Drivebase.GetDrivebase(), -8.7));
    }
}