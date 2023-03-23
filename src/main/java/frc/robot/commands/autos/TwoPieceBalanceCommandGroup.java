package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelCubeCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class TwoPieceBalanceCommandGroup extends SequentialCommandGroup
{
    public TwoPieceBalanceCommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new ParallelRaceGroup(new SetArmPositionCommand(Constants.ArmPose.HIGH_CUBE),new TimerCommand(1.5)),
              new ParallelRaceGroup(new ExpelCubeCommand(), new TimerCommand(.2)),
              new ParallelRaceGroup(new DriveDistanceCommandGyro(Drivebase.GetDrivebase(),10,.2))
              );

    }
}