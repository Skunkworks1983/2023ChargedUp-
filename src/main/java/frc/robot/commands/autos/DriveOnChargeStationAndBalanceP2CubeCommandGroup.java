package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;

public class DriveOnChargeStationAndBalanceP2CubeCommandGroup extends SequentialCommandGroup
{

    public DriveOnChargeStationAndBalanceP2CubeCommandGroup()
    {
        super(new SetArmRaceCommandGroup(Arm.ArmPosition.SCORE_MID, Arm.PostionPieceType.CUBE,1.75),
              new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2)),
              new SetArmRaceCommandGroup(Arm.ArmPosition.STOW, Arm.PostionPieceType.DOESNT_MATTER,.75),
              new DriveDistanceCommand(Drivebase.GetDrivebase(),-.5),
              new RotateCommand(Drivebase.GetDrivebase(),180),
              new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 11, Constants.Drivebase.DRIVEBASE_KF + .1),
              new RotateCommand(Drivebase.GetDrivebase(),180),
              new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 7, Constants.Drivebase.DRIVEBASE_KF + .15),
              new SafeBalanceCommandGroup());
    }
}