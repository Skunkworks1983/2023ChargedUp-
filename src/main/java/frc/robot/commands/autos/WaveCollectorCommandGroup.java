package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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