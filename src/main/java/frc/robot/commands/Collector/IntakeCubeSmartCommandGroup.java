package frc.robot.commands.Collector;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ConditionalStowCommand;
import frc.robot.commands.autos.TimerCommand;
import frc.robot.subsystems.Collector;

public class IntakeCubeSmartCommandGroup extends SequentialCommandGroup {
    private final Collector collector = Collector.getInstance();
    public IntakeCubeSmartCommandGroup() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                new IntakeCubeSmartCommand(),
                new ParallelRaceGroup(
                        new ConditionalStowCommand(),
                        new TimerCommand(.5)
                ),
                new SetCollectorHoldCommand()
        );

    }
}