package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.Collector.IntakeConeSmartCommand;
import frc.robot.commands.Collector.IntakeCubeSmartCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;

public class DriveOnChargeStationAndBalanceP2ConeCommandGroup extends SequentialCommandGroup
{

    public DriveOnChargeStationAndBalanceP2ConeCommandGroup()
    {
        super(new SetArmRaceCommandGroup(Arm.PoseType.SCORE, Constants.ArmPos.SCORE_CONE_MID_SHOULDER, Constants.ArmPos.SCORE_CONE_MID_WRIST,1.75),
              new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2)),
              new SetArmRaceCommandGroup(Arm.PoseType.RESTING, Constants.ArmPos.CARRY_SHOULDER,Constants.ArmPos.CARRY_WRIST,.75),
              new DriveDistanceCommand(Drivebase.GetDrivebase(),-.5),
              new RotateCommand(Drivebase.GetDrivebase(),180),
              new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 10, Constants.Drivebase.DRIVEBASE_KF + .1),
              new ParallelRaceGroup(new RotateCommand(Drivebase.GetDrivebase(), 180), new SetArmPositionCommand(Arm.PoseType.COLLECT, Constants.ArmPos.CONE_FLOOR_PICKUP_SHOULDER, Constants.ArmPos.CONE_FLOOR_PICKUP_WRIST )),
              new ParallelRaceGroup(new IntakeConeSmartCommand(),new DriveDistanceCommand(Drivebase.GetDrivebase(),-2)),
              new SetArmRaceCommandGroup(Arm.PoseType.RESTING, Constants.ArmPos.CARRY_SHOULDER,Constants.ArmPos.CARRY_WRIST, .7),
              /*new RotateCommand(Drivebase.GetDrivebase(),180),*/
              new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 7, Constants.Drivebase.DRIVEBASE_KF + .15),
              new SafeBalanceCommandGroup());
    }
}