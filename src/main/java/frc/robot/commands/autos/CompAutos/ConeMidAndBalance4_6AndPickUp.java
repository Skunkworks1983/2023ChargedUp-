package frc.robot.commands.autos.CompAutos;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.Collector.IntakeConeSmartCommand;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.autos.SafeBalanceCommandGroup;
import frc.robot.commands.autos.SetArmRaceCommandGroup;
import frc.robot.commands.autos.TimerCommand;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class ConeMidAndBalance4_6AndPickUp extends SequentialCommandGroup
{

    public ConeMidAndBalance4_6AndPickUp()
    {
        super(new SetArmRaceCommandGroup(Constants.ArmPos.SCORE_CONE_MID_SHOULDER, Constants.ArmPos.SCORE_CONE_MID_WRIST, 1.75),
              new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2)),
              new SetArmRaceCommandGroup(Constants.ArmPos.CARRY_SHOULDER,Constants.ArmPos.CARRY_WRIST,.75),
              new DriveDistanceCommand(Drivebase.GetDrivebase(),-.5),
              new RotateCommand(Drivebase.GetDrivebase(),180),
              new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 9, Constants.Drivebase.DRIVEBASE_KF + .1),
             new ParallelRaceGroup(new RotateCommand(Drivebase.GetDrivebase(),180),
                                   new SetArmPositionCommand(Constants.ArmPos.CONE_FLOOR_PICKUP_SHOULDER, Constants.ArmPos.CONE_FLOOR_PICKUP_WRIST)),
             new ParallelRaceGroup(new DriveDistanceCommand(Drivebase.GetDrivebase(),-5),new IntakeConeSmartCommand()),
             new ParallelRaceGroup(new DriveDistanceCommand(Drivebase.GetDrivebase(),6.5),
                                   new SetArmPositionCommand(Constants.ArmPos.CARRY_SHOULDER,Constants.ArmPos.CARRY_WRIST)),
              new ParallelCommandGroup(
                      new SafeBalanceCommandGroup(),
                      new ResetArm()
                    )
                );
    }
}