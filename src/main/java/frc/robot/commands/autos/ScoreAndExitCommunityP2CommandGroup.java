package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;


public class ScoreAndExitCommunityP2CommandGroup extends SequentialCommandGroup
{

    public ScoreAndExitCommunityP2CommandGroup()
    {
        super(new SetArmRaceCommandGroup(Constants.ArmPos.SCORE_CUBE_MID_SHOULDER, Constants.ArmPos.SCORE_CUBE_MID_WRIST, 1.75),
              new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2)),
              new SetArmRaceCommandGroup(Constants.ArmPos.CARRY_SHOULDER,Constants.ArmPos.CARRY_WRIST,.75),
              new DriveDistanceCommand(Drivebase.GetDrivebase(),-.5),
              new DriveDistanceCommand(Drivebase.GetDrivebase(), -1.58),
              new RotateCommand(Drivebase.GetDrivebase(), 90),
              new DriveDistanceCommand(Drivebase.GetDrivebase(), 5.75),
              new RotateCommand(Drivebase.GetDrivebase(), 90),
              new DriveDistanceCommand(Drivebase.GetDrivebase(), -11 ));
    }
}