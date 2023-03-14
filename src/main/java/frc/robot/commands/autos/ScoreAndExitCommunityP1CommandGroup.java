package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.subsystems.Drivebase;

public class ScoreAndExitCommunityP1CommandGroup extends SequentialCommandGroup
{
    //private static final Command

    public ScoreAndExitCommunityP1CommandGroup()
    {
        super(/*move arm and expel gp*/new DriveDistanceCommand(Drivebase.GetDrivebase(), 16.25));
    }
}