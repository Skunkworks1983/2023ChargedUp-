package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Collector.ExpelCubeCommand;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.subsystems.Drivebase;

public class ScoreAndDriveOutP3CommandGroup extends SequentialCommandGroup {
   private static final Command GrabCube4FromP3 = new GrabCube4FromP3CommandGroup();
   private static final Command Cube4ToScoreP3 = new Cube4ToScoreP3MidCommandGroup();
   private static final Command Expel = new ExpelCubeCommand(1);



    public ScoreAndDriveOutP3CommandGroup() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(GrabCube4FromP3, Cube4ToScoreP3, Expel);
    }
}