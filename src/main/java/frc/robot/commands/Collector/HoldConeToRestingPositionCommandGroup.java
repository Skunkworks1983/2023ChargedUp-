package frc.robot.commands.Collector;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class HoldConeToRestingPositionCommandGroup extends ParallelCommandGroup {
    public static final Command HoldCone = new HoldConeCommand();
    public static final Command ArmToCarry = new SetArmPositionCommand(Arm.PoseType.RESTING, Constants.Arm.SHOULDER_RESTING_ANGLE, Constants.ArmPos.CARRY_WRIST);

    public HoldConeToRestingPositionCommandGroup() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        if (Arm.getInstance().getCurrentPose() == Arm.PoseType.COLLECT) {
            addCommands(HoldCone, ArmToCarry);
        }
        else {
            addCommands(HoldCone);
        }
    }
}