package frc.robot.commands.autos.CompAutos;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.Collector.ExpelCubeCommand;
import frc.robot.commands.Collector.IntakeConeSmartCommand;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.autos.SafeBalanceCommandGroup;
import frc.robot.commands.autos.TimerCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class TwoPieceBalance8Red extends SequentialCommandGroup
{
    public TwoPieceBalance8Red()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new ParallelRaceGroup(
                new SetArmPositionCommand(Constants.ArmPose.HIGH_CUBE_AUTO),
                new TimerCommand(1.5)
              ),
              new ParallelRaceGroup(
                      new ExpelCubeCommand(),
                      new TimerCommand(.3)
              ),
              new ParallelRaceGroup(
                      new SetArmPositionCommand(Constants.ArmPose.STOW),
                      new TimerCommand(.3),
                      new RotateCommand(Drivebase.GetDrivebase(), -12, false)
              ),
              new ParallelCommandGroup(
                      new DriveDistanceCommandGyro(Drivebase.GetDrivebase(),-10.5,.525),
                      new SequentialCommandGroup(
                              new TimerCommand(2),
                              new ParallelRaceGroup(
                                      new SetArmPositionCommand(Constants.ArmPose.FLOOR_CONE),
                                      new TimerCommand(0.1)
                              )
                      )
              ) ,
              new ParallelRaceGroup(
                      new DriveDistanceCommandGyro(Drivebase.GetDrivebase(),-2.75,.0775),
                      new IntakeConeSmartCommand()
              ),
              new ParallelCommandGroup(
                      new DriveDistanceCommandGyro(Drivebase.GetDrivebase(),11.25,.565),
                      new ParallelRaceGroup(
                              new IntakeConeSmartCommand(),
                              new SetArmPositionCommand(Constants.ArmPose.STOW_AUTO),
                              new TimerCommand(1.5)
                      )
              ),
              new RotateCommand(Drivebase.GetDrivebase(), -140, false),
              new ParallelRaceGroup(
                      new ExpelConeCommand(),
                      new TimerCommand(.2)
              ),
              new ParallelRaceGroup(
                      new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 6.95, .5),
                      new SetArmPositionCommand(Constants.ArmPose.STOW)
              ),
              new ParallelCommandGroup(
                      new SafeBalanceCommandGroup(), new ResetArm()
              )
             );

    }
}