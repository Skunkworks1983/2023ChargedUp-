package frc.robot.commands.Collector;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.ScoreAndDriveOutP3CommandGroup;

public class IntakeConeAndHoldCommandGroup extends SequentialCommandGroup {
    private static final Command intakeConeSmart = new IntakeConeSmartCommand();
    private static final Command holdConeToRest = new HoldConeToRestingPositionCommandGroup();
    public IntakeConeAndHoldCommandGroup() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(intakeConeSmart, holdConeToRest);
    }
}