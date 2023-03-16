package frc.robot.commands.Collector;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class HoldConeToRestingPositionCommandGroup extends CommandBase {
    public static final Command HoldCone = new HoldConeCommand();
    public static final Command ArmToCarry = new SetArmPositionCommand(Arm.PoseType.RESTING, Constants.Arm.SHOULDER_RESTING_ANGLE, Constants.ArmPos.CARRY_WRIST);

    public HoldConeToRestingPositionCommandGroup() {
        super();
    }

    @Override
    public void initialize() {

        if (Arm.getInstance().getCurrentPose() == Arm.PoseType.COLLECT) {
            System.out.println("Hold cone and go to stow position");
            HoldCone.schedule();
            ArmToCarry.schedule();
        }
        else {
            System.out.println("it does Hold Cone ");

            HoldCone.schedule();
        }
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return HoldCone.isFinished() &&   ArmToCarry.isFinished();
    }

    @Override
    public void end(boolean interrupt)
    {
        HoldCone.end(interrupt);
        ArmToCarry.end(interrupt);
    }
}