package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.drivebase.DetectRangeSensorCommand;

public class SafeBalanceCommandGroup extends ParallelRaceGroup
{

    public SafeBalanceCommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new DetectRangeSensorCommand(), new BalanceOnChargeStationCommand(.023, 0, 0, .1));

    }
}