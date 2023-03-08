package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.subsystems.Drivebase;


public class ScoreAndExitCommunityP2CommandGroup extends SequentialCommandGroup
{
    private static final Command DriveDistanceCommand1 = new DriveDistanceCommand(Drivebase.GetDrivebase(), -1.58);
    private static final Command RotateCommand1 = new RotateCommand(Drivebase.GetDrivebase(), 90);
    private static final Command DriveDistanceCommand2 = new DriveDistanceCommand(Drivebase.GetDrivebase(),5.75);
    private static final Command RotateCommand2 = new RotateCommand(Drivebase.GetDrivebase(),90);
    private static final Command DriveDistanceCommand3 = new DriveDistanceCommand(Drivebase.GetDrivebase(),-11 );

    public ScoreAndExitCommunityP2CommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(DriveDistanceCommand1, RotateCommand1, DriveDistanceCommand2,RotateCommand2,DriveDistanceCommand3);
    }
}