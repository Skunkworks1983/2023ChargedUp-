package frc.robot.commands.autos.Simple;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.Collector.ExpelCubeCommand;
import frc.robot.commands.autos.SafeBalanceCommandGroup;
import frc.robot.commands.autos.SetArmRaceCommandGroup;
import frc.robot.commands.autos.TimerCommand;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;

public class SimpleAuto5 extends SequentialCommandGroup {
    public SimpleAuto5() {
        super(new SetArmRaceCommandGroup(Arm.PoseType.SCORE,Constants.ArmPos.SCORE_CUBE_MID_SHOULDER, Constants.ArmPos.SCORE_CUBE_MID_WRIST, 1.75),
                new ParallelRaceGroup(new ExpelCubeCommand(), new TimerCommand(.2)),
                new SetArmRaceCommandGroup(Arm.PoseType.RESTING,Constants.ArmPos.CARRY_SHOULDER, Constants.ArmPos.CARRY_WRIST, .75),
                new DriveDistanceCommand(Drivebase.GetDrivebase(), -.5),
                new RotateCommand(Drivebase.GetDrivebase(), 180),
                new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 11, Constants.Drivebase.DRIVEBASE_KF + .1),
                new RotateCommand(Drivebase.GetDrivebase(), 180),
                new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 7, Constants.Drivebase.DRIVEBASE_KF + .15),
                new SafeBalanceCommandGroup()
        );
    }
}