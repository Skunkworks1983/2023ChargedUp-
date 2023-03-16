package frc.robot.commands.autos.Simple;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.Collector.ExpelCubeCommand;
import frc.robot.commands.Collector.IntakeConeSmartCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.autos.SetArmRaceCommandGroup;
import frc.robot.commands.autos.TimerCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class SimpleAuto2_8 extends SequentialCommandGroup
{
    public SimpleAuto2_8()
    {
        super
                (
                        new SetArmRaceCommandGroup(Constants.ArmPos.SCORE_CUBE_MID_SHOULDER, Constants.ArmPos.SCORE_CUBE_MID_WRIST, 2),
                        new ParallelRaceGroup(new ExpelCubeCommand(), new TimerCommand(.2)),
                        new ParallelRaceGroup
                                (
                                        new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), -11.5, Constants.Drivebase.DRIVEBASE_KF + .07),
                                        new SetArmPositionCommand(Constants.ArmPos.FLOOR_CUBE_PICKUP_SHOULDER, Constants.ArmPos.FLOOR_CUBE_PICKUP_WRIST)
                                ),
                        new ParallelRaceGroup
                                (
                                        new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), -3, Constants.Drivebase.DRIVEBASE_KF - 0.05),
                                        new IntakeConeSmartCommand()
                                ),
                        new ParallelRaceGroup
                                (
                                        new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 11.5, Constants.Drivebase.DRIVEBASE_KF + .07),
                                        new SetArmPositionCommand(Constants.ArmPos.CARRY_SHOULDER, Constants.ArmPos.CARRY_WRIST)
                                ),
                        new RotateCommand(Drivebase.GetDrivebase(), 180),
                        new SetArmRaceCommandGroup(Constants.ArmPos.FLOOR_NORMAL_SCORE_SHOULDER, Constants.ArmPos.FLOOR_NORMAL_SCORE_WRIST, 2),
                        new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2))
                );
    }
}
