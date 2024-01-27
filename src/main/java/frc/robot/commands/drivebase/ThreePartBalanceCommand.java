package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.BalanceOnChargeStationCommand;
import frc.robot.commands.autos.SafeBalanceCommandGroup;
import frc.robot.subsystems.Drivebase;


public class ThreePartBalanceCommand extends SequentialCommandGroup {


    public ThreePartBalanceCommand()
    {
        super(new DriveUntilChargeStationCommand(.3,16),
                new DriveDistanceCommandGyro(Drivebase.GetDrivebase(),3,.2),
                new SafeBalanceCommandGroup()
        );
        //super(new DetectRangeSensorCommand(), new BalanceOnChargeStationCommand(.023, 0.017, 0, .15));
    }
}
