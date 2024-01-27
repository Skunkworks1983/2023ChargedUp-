package frc.robot.commands.drivebase;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivebase;

public class TestAutoTerminateCommandGroup extends SequentialCommandGroup
{
    public TestAutoTerminateCommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                new ParallelRaceGroup(
                        new DriveDistanceCommandGyro(Drivebase.GetDrivebase(), 10, 0.1),
                        new PoseEstimatorTerminateCommand(5)
                )
             );
    }
}