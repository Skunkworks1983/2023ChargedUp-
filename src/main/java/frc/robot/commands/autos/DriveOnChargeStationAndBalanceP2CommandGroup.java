package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.subsystems.Drivebase;

public class DriveOnChargeStationAndBalanceP2CommandGroup extends SequentialCommandGroup
{
    private static final Command DriveDistanceCommandForward = new DriveDistanceCommand(Drivebase.GetDrivebase(),-14.5);
    private static final Command DriveDistanceCommandBackward = new DriveDistanceCommand(Drivebase.GetDrivebase(),7);
    public DriveOnChargeStationAndBalanceP2CommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(DriveDistanceCommandForward, DriveDistanceCommandBackward
                /*add balance code from Eleanor*/);
    }
}