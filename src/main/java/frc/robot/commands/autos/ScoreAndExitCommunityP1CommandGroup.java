package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.subsystems.Drivebase;

public class ScoreAndExitCommunityP1CommandGroup extends SequentialCommandGroup
{
    private static final Command DriveDistanceCommand = new DriveDistanceCommand(Drivebase.GetDrivebase(), 16.25);
    //private static final Command

    public ScoreAndExitCommunityP1CommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(/*move arm and expel gp*/DriveDistanceCommand);
    }
}