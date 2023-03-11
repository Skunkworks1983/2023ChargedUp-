package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelConeCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.constants.Constants;

public class PlaceGpInAutoConeCommandGroup extends SequentialCommandGroup
{
    private static final Command SetArmPositionCommand = new SetArmPositionCommand(Constants.ArmPos.SCORE_CONE_MID_SHOULDER, Constants.ArmPos.SCORE_CONE_MID_WRIST);
    private static final Command ExpelConeCommand = new ExpelConeCommand();

    public PlaceGpInAutoConeCommandGroup()
    {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(SetArmPositionCommand, ExpelConeCommand);
    }
}