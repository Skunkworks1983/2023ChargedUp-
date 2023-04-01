package frc.robot.commands.autos;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.Collector.ExpelCubeCommand;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class TestAutoTwoCommandGroup/*two peice auto*/ extends SequentialCommandGroup {
    public TestAutoTwoCommandGroup() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());

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
                        new SmartDriveCommand(Constants.Autos.FirstAuto.trajectoryOne)
                )
                //drive and collect command

                ,
                new SmartDriveCommand(Constants.Autos.FirstAuto.trajectoryTwo),

                new SetArmRaceCommandGroup(Constants.ArmPose.SCORE_MID_CONE, 1.5),
                new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2)),


                new SmartDriveCommand(Constants.Autos.FirstAuto.trajectoryThree),

                new SmartDriveCommand(Constants.Autos.FirstAuto.trajectoryFour),

                new ParallelCommandGroup(
                        new SafeBalanceCommandGroup(), new ResetArm()
                )

        );

    }
}