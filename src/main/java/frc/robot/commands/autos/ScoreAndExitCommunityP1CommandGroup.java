package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;

public class ScoreAndExitCommunityP1CommandGroup extends SequentialCommandGroup
{
    //private static final Command

    public ScoreAndExitCommunityP1CommandGroup()
    {
        super(new SetArmRaceCommandGroup(Constants.ArmPose.SCORE_MID_CUBE, 1.75),
              new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2)),
              new SetArmRaceCommandGroup(Constants.ArmPose.STOW,.75),
              new DriveDistanceCommand(Drivebase.GetDrivebase(),-.5),
              new ParallelCommandGroup(new DriveDistanceCommand(Drivebase.GetDrivebase(), 16.25), new ResetArm())
             );
    }
}