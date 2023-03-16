package frc.robot.commands.autos.Simple;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.Collector.IntakeConeSmartCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.autos.SafeBalanceCommandGroup;
import frc.robot.commands.autos.SetArmRaceCommandGroup;
import frc.robot.commands.autos.TimerCommand;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;


public class SimpleAuto3_7 extends SequentialCommandGroup
{

    public SimpleAuto3_7()
    {
        super(new SetArmRaceCommandGroup(Constants.ArmPos.SCORE_CONE_MID_SHOULDER, Constants.ArmPos.SCORE_CONE_MID_WRIST,1.75), // set arm position mid scoring thing
                new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2)), //expel cone
                new SetArmRaceCommandGroup(Constants.ArmPos.CARRY_SHOULDER,Constants.ArmPos.CARRY_WRIST,.75), // set arm back into carry position
                new DriveDistanceCommand(Drivebase.GetDrivebase(),-.5), // drive backwards a little
                new RotateCommand(Drivebase.GetDrivebase(),-90), // spin 180 degrees
                new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 10, Constants.Drivebase.DRIVEBASE_KF + .1), //drives over charging station
                new
    }
}
