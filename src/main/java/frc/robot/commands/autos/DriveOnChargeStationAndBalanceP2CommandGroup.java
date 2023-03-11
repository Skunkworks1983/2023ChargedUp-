package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class DriveOnChargeStationAndBalanceP2CommandGroup extends SequentialCommandGroup
{

    public DriveOnChargeStationAndBalanceP2CommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(/*new PlaceGpInAutoConeCommandGroup(),*/ new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), -14, Constants.Drivebase.DRIVEBASE_KF), new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 7, Constants.Drivebase.DRIVEBASE_KF + .02)
                /*add balance code from Eleanor*/);
    }
}