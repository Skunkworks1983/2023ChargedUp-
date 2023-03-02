package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.arm.SetShoulderSpeed;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class LiftArmAndWaveAutoCommandGroup extends ParallelRaceGroup
{
    private static final Command WaveCollectorCommandGroup = new WaveCollectorCommandGroup();
    private static final Command SetShoulderSpeed = new SetShoulderSpeed(Arm.getInstance(), Constants.Arm.SHOULDER_LIMIT_SWITCH_FRONT, 0.08);

    public LiftArmAndWaveAutoCommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(WaveCollectorCommandGroup, SetShoulderSpeed);
    }
}