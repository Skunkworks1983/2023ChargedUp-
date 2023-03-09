package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Collector.IntakeCubeSmartCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.arm.WristRotateDegrees;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class GrabCube4FromP3CommandGroup extends ParallelCommandGroup {
    private static final Command DriveDistanceCommand = new DriveDistanceCommand(Drivebase.GetDrivebase(), -15.75);
    //private static final Command WristMoveCommand = new SetArmPositionCommand
      //      (Constants.ArmPos.FLOOR_CUBE_PICKUP_SHOULDER, Constants.ArmPos.FLOOR_CUBE_PICKUP_WRIST);
    //private static final Command IntakeCommand=new IntakeCubeSmartCommand();
    public GrabCube4FromP3CommandGroup() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(DriveDistanceCommand);
    }
}