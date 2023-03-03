package frc.robot.commands.arm;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.TimeWristMoveCommandGroup;

public class WaveCollectorCommandGroup extends SequentialCommandGroup {
    public WaveCollectorCommandGroup() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());

        super(new TimeWristMoveCommandGroup(2, 0.15),
                new TimeWristMoveCommandGroup(.9,-.14),
                new TimeWristMoveCommandGroup(.9,.14),
                new TimeWristMoveCommandGroup(.9,-.14),
                new TimeWristMoveCommandGroup(.9,.14));
    }
}