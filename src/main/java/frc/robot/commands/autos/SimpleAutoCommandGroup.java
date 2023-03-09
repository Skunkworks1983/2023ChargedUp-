package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.subsystems.Drivebase;

public class SimpleAutoCommandGroup extends SequentialCommandGroup
{
    //private static final Command RotateCommand = new RotateCommand(Drivebase.GetDrivebase(),180);

    public SimpleAutoCommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new DriveDistanceCommand(Drivebase.GetDrivebase(), 1 ), new DriveDistanceCommand(Drivebase.GetDrivebase(), -1), new LiftArmAndWaveAutoCommandGroup()/*, RotateCommand*/);
    }
}