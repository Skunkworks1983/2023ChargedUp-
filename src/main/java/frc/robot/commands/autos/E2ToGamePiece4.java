package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.RotateWristByPowerCommand;
import frc.robot.commands.arm.WaveCollectorCommandGroup;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.commands.drivebase.RotateCommand;
import frc.robot.subsystems.Drivebase;
import frc.robot.commands.arm.WaveCollectorCommandGroup;

public class E2ToGamePiece4 extends ParallelCommandGroup
{
    public E2ToGamePiece4()
    {
        super(new DriveDistanceCommand(Drivebase.GetDrivebase(), -9.75)); //drive straight to 4th game piece
                //new );
    }
}
