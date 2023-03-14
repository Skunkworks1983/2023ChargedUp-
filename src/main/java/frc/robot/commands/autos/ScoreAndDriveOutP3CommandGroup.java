package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.subsystems.Drivebase;

public class ScoreAndDriveOutP3CommandGroup extends SequentialCommandGroup
{
    private static final Command DriveDistanceCommand = new DriveDistanceCommand(Drivebase.GetDrivebase(), -8.7);

    public ScoreAndDriveOutP3CommandGroup() {
        super(DriveDistanceCommand);
    }
}