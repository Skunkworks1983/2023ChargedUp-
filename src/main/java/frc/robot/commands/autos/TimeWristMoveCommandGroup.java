package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.TimerCommand;

public class TimeWristMoveCommandGroup extends ParallelRaceGroup {
    public TimeWristMoveCommandGroup(double seconds,double speed) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new TimerCommand(seconds),new RotateWristByPowerCommand(speed));
    }
}