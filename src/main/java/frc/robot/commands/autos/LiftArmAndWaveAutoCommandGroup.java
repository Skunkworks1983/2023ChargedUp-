package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

public class LiftArmAndWaveAutoCommandGroup extends ParallelRaceGroup
{
    private static final Command WaveCollectorCommandGroup = new WaveCollectorCommandGroup();

    public LiftArmAndWaveAutoCommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(WaveCollectorCommandGroup);
    }
}