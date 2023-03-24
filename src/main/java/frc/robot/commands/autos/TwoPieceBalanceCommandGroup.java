package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.Collector.ExpelCubeCommand;
import frc.robot.commands.Collector.IntakeConeSmartCommand;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class TwoPieceBalanceCommandGroup extends SequentialCommandGroup
{
    public TwoPieceBalanceCommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new ParallelRaceGroup(new SetArmPositionCommand(Constants.ArmPose.HIGH_CUBE),new TimerCommand(1.25)),
              new ParallelRaceGroup(new ExpelCubeCommand(), new TimerCommand(.3)),
              new ParallelRaceGroup(new SetArmPositionCommand(Constants.ArmPose.FLOOR_CONE),new TimerCommand(.3)),
              new DriveDistanceCommandGyro(Drivebase.GetDrivebase(),-10.5,.525),
              new ParallelRaceGroup(new DriveDistanceCommandGyro(Drivebase.GetDrivebase(),-2.5,.0775),new IntakeConeSmartCommand()),
              new ParallelCommandGroup(new ParallelRaceGroup(new IntakeConeSmartCommand(), new SetArmPositionCommand(Constants.ArmPose.STOW), new TimerCommand(1.5))),
              new ParallelRaceGroup(new DriveDistanceCommandGyro(Drivebase.GetDrivebase(),12.85,.565), new SetArmPositionCommand(Constants.ArmPose.FLOOR_NORMAL)),
              new RotateCommand(Drivebase.GetDrivebase(), 180, false),
              new ExpelConeCommand(),
              new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2)),
              new ParallelRaceGroup(new RotateCommand(Drivebase.GetDrivebase(), 43), new SetArmPositionCommand(Constants.ArmPose.STOW)),
              new ParallelRaceGroup(new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 6.95, .5), new SetArmPositionCommand(Constants.ArmPose.STOW)),
              new ParallelCommandGroup(new SafeBalanceCommandGroup(), new ResetArm())
             );

    }
}