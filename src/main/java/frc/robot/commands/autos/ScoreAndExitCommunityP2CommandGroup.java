package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.subsystems.Drivebase;


public class ScoreAndExitCommunityP2CommandGroup extends SequentialCommandGroup
{

    public ScoreAndExitCommunityP2CommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new DriveDistanceCommand(Drivebase.GetDrivebase(), -1.58), new RotateCommand(Drivebase.GetDrivebase(), 90), new DriveDistanceCommand(Drivebase.GetDrivebase(), 5.75), new RotateCommand(Drivebase.GetDrivebase(), 90), new DriveDistanceCommand(Drivebase.GetDrivebase(), -11 ));
    }
}