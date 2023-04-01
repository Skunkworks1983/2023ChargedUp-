package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.Collector.ExpelCubeCommand;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.constants.Constants;

public class TrajectoryTwoPieceBalanceAuto/*two peice auto*/ extends SequentialCommandGroup {
    public TrajectoryTwoPieceBalanceAuto() {

        super(
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
                        new TimerCommand(7.3),
                        new SmartDriveCommand(Constants.Autos.twoPeiceBalanceAuto.driveToObject)
                )
                //drive and collect command

                ,
                new SmartDriveCommand(Constants.Autos.twoPeiceBalanceAuto.driveToGrid),

                new SetArmRaceCommandGroup(Constants.ArmPose.SCORE_MID_CONE, 1.5),
                new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2)),


                new SmartDriveCommand(Constants.Autos.twoPeiceBalanceAuto.turnToBalance),

                new SmartDriveCommand(Constants.Autos.twoPeiceBalanceAuto.driveToBalance),

                new ParallelCommandGroup(
                        new SafeBalanceCommandGroup(), new ResetArm()
                )

        );

    }
}