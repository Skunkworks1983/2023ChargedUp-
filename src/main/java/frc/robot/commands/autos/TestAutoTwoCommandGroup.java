package frc.robot.commands.autos;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class TestAutoTwoCommandGroup extends SequentialCommandGroup {
    public TestAutoTwoCommandGroup() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());

        super(new SmartDriveCommand(Constants.Autos.FirstAuto.trajectoryOne),//would it be .scedual
                new SmartDriveCommand(Constants.Autos.FirstAuto.trajectoryTwo),
                new SmartDriveCommand(Constants.Autos.FirstAuto.trajectoryThree));

    }
}