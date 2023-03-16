package frc.robot.commands.autos.Simple;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.autos.SetArmRaceCommandGroup;
import frc.robot.commands.autos.TimerCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class SimpleAuto9 extends SequentialCommandGroup {
    public SimpleAuto9() {
        super(
                new SetArmRaceCommandGroup(Constants.ArmPos.SCORE_CONE_MID_SHOULDER, Constants.ArmPos.SCORE_CONE_MID_WRIST, 3),
                new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2)),
                new ParallelCommandGroup(new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), -11.5, Constants.Drivebase.DRIVEBASE_KF + .07), new SetArmPositionCommand(Constants.ArmPos.CARRY_SHOULDER, Constants.ArmPos.CARRY_WRIST))
        );
    }
}
