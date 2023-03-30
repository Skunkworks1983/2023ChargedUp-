package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;


public class IntakeConeAndStowCommand extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final Collector collector = Collector.getInstance();
    private final Command IntakeCone = new IntakeConeSmartCommand();
    private final Command StowCommand = new SetArmPositionCommand(Constants.ArmPose.STOW);
    public IntakeConeAndStowCommand() {

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize()
    {
        System.out.println("Intake and stow Initialized");
        IntakeCone.schedule();
    }

    @Override
    public void execute() {
        if(IntakeCone.isFinished() && arm.getCurrentPose().shouldAutoStow)
        {
            StowCommand.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return IntakeCone.isFinished();
    }

    @Override
    public void end(boolean interrupted)
    {
        System.out.println("Intake and stow End");
        IntakeCone.cancel();
        StowCommand.cancel();
    }
}
