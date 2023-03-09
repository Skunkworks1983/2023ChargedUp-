package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.subsystems.Drivebase;

public class DriveOnChargeStationAndBalanceP2CommandGroup extends SequentialCommandGroup
{
    public DriveOnChargeStationAndBalanceP2CommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new DriveDistanceCommand(Drivebase.GetDrivebase(), -14.5), new DriveDistanceCommand(Drivebase.GetDrivebase(), 7)
                /*add balance code from Eleanor*/);
    }
}