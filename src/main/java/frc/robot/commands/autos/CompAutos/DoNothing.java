package frc.robot.commands.autos.CompAutos;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetLightsCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class DoNothing extends SequentialCommandGroup
{
    public DoNothing()
    {
        super(
                new SetLightsCommand(Constants.Lights.PARTY)
             );
    }

}
