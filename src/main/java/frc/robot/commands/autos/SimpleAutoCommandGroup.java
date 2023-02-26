package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.subsystems.multidrivebase.Drivebase4MotorTalonFX;

public class SimpleAutoCommandGroup extends SequentialCommandGroup
{
    private static final Command DriveDistanceCommandForward = new DriveDistanceCommand(Drivebase4MotorTalonFX.GetDrivebase(), 1 );
    private static final Command DriveDistanceCommandBackward = new DriveDistanceCommand(Drivebase4MotorTalonFX.GetDrivebase(),-1);
    private static final Command LiftArmAndWaveAutoCommandGroup = new LiftArmAndWaveAutoCommandGroup();
    // private static final Command RotateCommand = new RotateCommand(Drivebase4MotorTalonFX.GetDrivebase(),180);

    public SimpleAutoCommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(DriveDistanceCommandForward, DriveDistanceCommandBackward,LiftArmAndWaveAutoCommandGroup/*, RotateCommand*/);
    }
}