package frc.robot.commands.autos;


        import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
        import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
        import frc.robot.commands.arm.ShoulderRotateDegrees;
        import frc.robot.commands.arm.WristRotateDegrees;
        import frc.robot.commands.drivebase.DriveDistanceCommand;
        import frc.robot.constants.Constants;
        import frc.robot.subsystems.Arm;

public class PositionShoulderAndWrist extends ParallelCommandGroup
{
    public PositionShoulderAndWrist(Arm arm, double wristDegrees, double shoulderDegrees)
    {
        super(new ShoulderRotateDegrees(arm, shoulderDegrees, true), new WristRotateDegrees(arm, wristDegrees, true));
    }
}