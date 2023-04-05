package frc.robot.commands.autos;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.Collector.ExpelCubeCommand;
import frc.robot.commands.Collector.IntakeConeSmartCommand;
import frc.robot.commands.Collector.IntakeCubeSmartCommand;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.commands.drivebase.ResetPoseCommand;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import pabeles.concurrency.ConcurrencyOps;

public class TrajectoryTwoPieceBalanceAuto/*two peice auto*/ extends SequentialCommandGroup {
    public TrajectoryTwoPieceBalanceAuto() {

        super(
                new ResetPoseCommand(Constants.Autos.twoPeiceBalanceAuto.startPose),
                new ParallelRaceGroup(
                    new SetArmPositionCommand(Constants.ArmPose.HIGH_CUBE_AUTO),
                    new TimerCommand(1.5)
                ),

                new ParallelRaceGroup(
                        new ExpelCubeCommand(),
                        new TimerCommand(.3)
                ),

                new ParallelRaceGroup(
                        new SetArmPositionCommand(Constants.ArmPose.STOW),
                        new TimerCommand(.75)
                ),

                new ParallelRaceGroup(
                    new SmartDriveCommand(Constants.Autos.twoPeiceBalanceAuto.driveToObject.concatenate(Constants.Autos.twoPeiceBalanceAuto.driveToObject2)),
                    new SequentialCommandGroup(
                            new TimerCommand(1.75),
                            new ParallelRaceGroup(
                                    new SetArmPositionCommand(Constants.ArmPose.FLOOR_CONE),
                                    new TimerCommand(0.5)
                            ),
                            new IntakeConeSmartCommand()
                    )
                ),
                new ParallelCommandGroup(
                    new SmartDriveCommand(Constants.Autos.twoPeiceBalanceAuto.driveToGrid),
                    new ParallelRaceGroup(
                        new SetArmPositionCommand(Constants.ArmPose.STOW),
                        new IntakeConeSmartCommand(),
                        new TimerCommand(1)
                        )
                    ),
                new RotateCommand(Drivebase.GetDrivebase(), -45),
                new ParallelRaceGroup(
                        new ExpelConeCommand(),
                        new TimerCommand(0.2)
                ),
                new ParallelRaceGroup(
                        new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 5.5, Constants.Drivebase.DRIVEBASE_KF + .32)
                        ,
                        new TimerCommand(4)
                )
                ,
                new ParallelCommandGroup(
                        new SafeBalanceCommandGroup(),
                        new ResetArm()
                )
        );

    }
}