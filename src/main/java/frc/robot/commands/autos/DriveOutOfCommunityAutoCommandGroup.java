package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class DriveOutOfCommunityAutoCommandGroup extends SequentialCommandGroup
{
    public DriveOutOfCommunityAutoCommandGroup(Drivebase drivebase)
    {
        super(new DriveDistanceCommand(drivebase, Constants.Drivebase.DRIVE_OUT_OF_COMMUNITY));
    }
}