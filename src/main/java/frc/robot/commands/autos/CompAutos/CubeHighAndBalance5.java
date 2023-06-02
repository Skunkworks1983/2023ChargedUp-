package frc.robot.commands.autos.CompAutos;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
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

public class CubeHighAndBalance5 extends SequentialCommandGroup {
    public CubeHighAndBalance5() {
        super(new SetArmRaceCommandGroup(Constants.ArmPose.HIGH_CUBE_AUTO, 1.5),
              new ParallelRaceGroup(new ExpelCubeCommand(), new TimerCommand(.2)),
              new SetArmRaceCommandGroup(Constants.ArmPose.HIGH_CUBE.STOW, .75),
              new DriveDistanceCommand(Drivebase.GetDrivebase(), -.5),
              new RotateCommand(Drivebase.GetDrivebase(), 180),
              new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 8.8, Constants.Drivebase.DRIVEBASE_KF + .22),
              new RotateCommand(Drivebase.GetDrivebase(), 180),
                new ParallelCommandGroup(new ResetArm(),
                        new ThreePartBalanceCommand())
              //new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 4.5, Constants.Drivebase.DRIVEBASE_KF + .15),
              //
             );
    }
}