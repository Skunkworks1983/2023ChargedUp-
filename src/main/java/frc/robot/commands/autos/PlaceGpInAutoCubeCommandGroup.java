package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.Collector.ExpelCubeCommand;
import frc.robot.commands.arm.SetArmPositionCommand;

public class PlaceGpInAutoCubeCommandGroup extends SequentialCommandGroup
{
  // private static final Command SetArmPositionCommand = new SetArmPositionCommand( , );
    private static final Command ExpelCubeCommand = new ExpelCubeCommand();

    public PlaceGpInAutoCubeCommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(/*SetArmPositionCommand,*/ExpelCubeCommand);
    }
}