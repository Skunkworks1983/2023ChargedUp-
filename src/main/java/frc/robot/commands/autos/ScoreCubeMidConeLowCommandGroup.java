package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class ScoreCubeMidConeLowCommandGroup extends SequentialCommandGroup
{
    public ScoreCubeMidConeLowCommandGroup()
    {
        //super(
        //        SetArmRaceCommandGroup(Constants.ArmPos.sco)
        //      )
    }
}
