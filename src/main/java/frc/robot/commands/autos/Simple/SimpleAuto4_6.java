

package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.Collector.IntakeConeSmartCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.DriveDistanceCommandGyro;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;


public class SimpleAuto4_6 extends SequentialCommandGroup
{

    public SimpleAuto4_6()
    {
        super(new SetArmRaceCommandGroup(Constants.ArmPos.SCORE_CONE_MID_SHOULDER, Constants.ArmPos.SCORE_CONE_MID_WRIST,1.75), // set arm position mid scoring thing
                new ParallelRaceGroup(new ExpelConeCommand(), new TimerCommand(.2)), //expel cone
                new SetArmRaceCommandGroup(Constants.ArmPos.CARRY_SHOULDER,Constants.ArmPos.CARRY_WRIST,.75), // set arm back into carry position
                new DriveDistanceCommand(Drivebase.GetDrivebase(),-.5), // drive backwards a little
                new RotateCommand(Drivebase.GetDrivebase(),180), // spin 180 degrees
                new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 10, Constants.Drivebase.DRIVEBASE_KF + .1), //drives over charging station
                new ParallelRaceGroup(new RotateCommand(Drivebase.GetDrivebase(), 180), new SetArmPositionCommand(Constants.ArmPos.CONE_FLOOR_PICKUP_SHOULDER, Constants.ArmPos.CONE_FLOOR_PICKUP_WRIST )), //drives to where cone is
                new ParallelRaceGroup(new IntakeConeSmartCommand(),new DriveDistanceCommand(Drivebase.GetDrivebase(),-2)), //picks cone up off floor
                new SetArmRaceCommandGroup(Constants.ArmPos.CARRY_SHOULDER,Constants.ArmPos.CARRY_WRIST, .7), // sets arm in carry position
                /*new RotateCommand(Drivebase.GetDrivebase(),180),*/
                new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 7, Constants.Drivebase.DRIVEBASE_KF + .15), // drives up to charging station
                new SafeBalanceCommandGroup()); //balances on charging station
    }
}