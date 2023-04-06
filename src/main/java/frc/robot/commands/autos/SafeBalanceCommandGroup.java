package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.drivebase.DetectRangeSensorCommand;

public class SafeBalanceCommandGroup extends ParallelRaceGroup
{

    public SafeBalanceCommandGroup()
    {
        super(new DetectRangeSensorCommand(), new BalanceOnChargeStationCommand(.023, 0.004, 0, .09));
        //super(new DetectRangeSensorCommand(), new BalanceOnChargeStationCommand(.023, 0.05, 0, .15));
    }
}