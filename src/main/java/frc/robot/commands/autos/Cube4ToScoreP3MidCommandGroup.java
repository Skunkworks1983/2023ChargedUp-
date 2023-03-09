package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class Cube4ToScoreP3MidCommandGroup extends ParallelCommandGroup {
    private static final Command driveDistanceCommand = new DriveDistanceCommand(Drivebase.GetDrivebase(), 15.75);
    //private static final Command armPosition = new SetArmPositionCommand
            //(Constants.ArmPos.SCORE_CUBE_MID_SHOULDER, Constants.ArmPos.SCORE_CUBE_MID_WRIST);

    public Cube4ToScoreP3MidCommandGroup() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(driveDistanceCommand);
    }
}