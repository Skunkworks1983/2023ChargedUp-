package frc.robot.commands.autos.CompAutos;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelCubeCommand;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.autos.SafeBalanceCommandGroup;
import frc.robot.commands.autos.SetArmRaceCommandGroup;
import frc.robot.commands.autos.TimerCommand;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.commands.drivebase.ThreePartBalanceCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;

public class CubeMidAndBalance5 extends SequentialCommandGroup {
    public CubeMidAndBalance5() {
        super(new SetArmRaceCommandGroup(Constants.ArmPose.SCORE_MID_CUBE, 1.5),
                new ParallelRaceGroup(new ExpelCubeCommand(), new TimerCommand(.2)),
                new SetArmRaceCommandGroup(Constants.ArmPose.STOW, .75),
                new DriveDistanceCommand(Drivebase.GetDrivebase(), -.5),
                new RotateCommand(Drivebase.GetDrivebase(), 180),
                new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 11, Constants.Drivebase.DRIVEBASE_KF + .1),
                new RotateCommand(Drivebase.GetDrivebase(), 180),
                //new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 6.5, Constants.Drivebase.DRIVEBASE_KF + .15),
                new ParallelCommandGroup(new ThreePartBalanceCommand(), new ResetArm())
        );
    }
}