package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.arm.SetShoulderSpeed;
import frc.robot.commands.arm.WaveCollectorCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class LiftArmAndWaveAutoCommandGroup extends ParallelRaceGroup
{

    public LiftArmAndWaveAutoCommandGroup()
    {
        super(new WaveCollectorCommandGroup(), new SetShoulderSpeed(Arm.getInstance(), Constants.Arm.SHOULDER_LIMIT_SWITCH_FRONT, 0.08));
    }
}