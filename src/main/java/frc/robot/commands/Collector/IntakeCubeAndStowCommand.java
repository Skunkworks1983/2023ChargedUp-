package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;


public class IntakeCubeAndStowCommand extends Command {
    private final Arm arm = Arm.getInstance();
    private final Collector collector = Collector.getInstance();
    private final Command IntakeCube = new IntakeCubeSmartCommand();
    private final Command StowCommand = new SetArmPositionCommand(Constants.ArmPose.STOW);
    public IntakeCubeAndStowCommand() {

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
        IntakeCube.schedule();
    }

    @Override
    public void execute() {
        if(IntakeCube.isFinished() && arm.getCurrentPose().shouldAutoStow)
        {
            StowCommand.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return IntakeCube.isFinished() && StowCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        IntakeCube.cancel();
        StowCommand.cancel();
    }
}
