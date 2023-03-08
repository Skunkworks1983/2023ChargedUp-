package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.subsystems.Drivebase;


public class LeaveCommunityP2E2 extends SequentialCommandGroup
{

    public LeaveCommunityP2E2()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new DriveDistanceCommand(Drivebase.GetDrivebase(), -1.8), // drive away from p2.
                new RotateCommand(Drivebase.GetDrivebase(), 90),                     // turn left.
                new DriveDistanceCommand(Drivebase.GetDrivebase(),-5.7),          // drive left.
                new RotateCommand(Drivebase.GetDrivebase(),-90),                     // turn right.
                new DriveDistanceCommand(Drivebase.GetDrivebase(), -4.8));        // drive straight to e2.
    }
}