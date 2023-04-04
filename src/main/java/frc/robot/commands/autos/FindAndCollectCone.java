package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.IntakeConeSmartCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.drivebase.DriveToGamePieceCommand;
import frc.robot.constants.Constants;

public class FindAndCollectCone extends SequentialCommandGroup {
    public FindAndCollectCone() {
        super(

                new ParallelRaceGroup(
                        new DriveToGamePieceCommand(-0.15, 10),
                        new IntakeConeSmartCommand()
                ),
                new ParallelRaceGroup(
                        new SetArmPositionCommand(Constants.ArmPose.STOW_AUTO),
                        new TimerCommand(0.5)
                )
        );
    }
}