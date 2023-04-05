package frc.robot.commands.Collector;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.constants.Constants;

public class ScoreMidConeAndHoldCommandGroup extends ParallelCommandGroup {
    public ScoreMidConeAndHoldCommandGroup() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new SetHoldModeCommand(), new SetArmPositionCommand(Constants.ArmPose.SCORE_MID_CONE));
    }
}