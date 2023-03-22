package frc.robot.commands.autos.CompAutos;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.autos.SafeBalanceCommandGroup;
import frc.robot.commands.autos.SetArmRaceCommandGroup;
import frc.robot.commands.autos.TimerCommand;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class ConeMidAndBalance4_6 extends SequentialCommandGroup
{

    public ConeMidAndBalance4_6()
    {
        super(new SetArmRaceCommandGroup(Constants.ArmPos.SCORE_CONE_MID_SHOULDER, Constants.ArmPos.SCORE_CONE_MID_WRIST, 1.75),
              new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2)),
              new SetArmRaceCommandGroup(Constants.ArmPos.CARRY_SHOULDER,Constants.ArmPos.CARRY_WRIST,.75),
              new DriveDistanceCommand(Drivebase.GetDrivebase(),-.5),
              new RotateCommand(Drivebase.GetDrivebase(),180),
              new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 10, Constants.Drivebase.DRIVEBASE_KF + .1),
              new RotateCommand(Drivebase.GetDrivebase(),180),
              new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 6.5, Constants.Drivebase.DRIVEBASE_KF + .15),
              new ParallelCommandGroup(
                      new SafeBalanceCommandGroup(),
                      new ResetArm()
                    )
                );
    }
}