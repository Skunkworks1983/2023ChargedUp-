package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class SetLightsCommand extends Command {

    Arm arm = Arm.getInstance();
    int lightMode;

    public SetLightsCommand(int lightMode)
    {
        this.lightMode = lightMode;
    }

    @Override
    public void initialize()
    {
        arm.SetLightMode(lightMode);
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        arm.SetLightMode(Constants.Lights.BLANK);
    }
}
