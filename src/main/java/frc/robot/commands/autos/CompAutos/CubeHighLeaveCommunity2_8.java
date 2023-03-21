package frc.robot.commands.autos.CompAutos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.autos.SetArmRaceCommandGroup;
import frc.robot.commands.autos.TimerCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;

public class CubeHighLeaveCommunity2_8 extends SequentialCommandGroup
{
    public CubeHighLeaveCommunity2_8()
    {
        super
                (
                        new SetArmRaceCommandGroup(Constants.ArmPos.SCORE_CUBE_HIGH_SHOULDER, Constants.ArmPos.SCORE_CUBE_HIGH_WRIST, 2),
                        new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2)),
                        new ParallelRaceGroup
                                (
                                        new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), -12, Constants.Drivebase.DRIVEBASE_KF + .07),
                                        new SetArmPositionCommand(Arm.ArmPosition.STOW)
                                ),
                        new ResetArm()
                );
    }
}
