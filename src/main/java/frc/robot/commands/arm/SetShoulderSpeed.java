package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class SetShoulderSpeed extends CommandBase {

    private final Arm arm;
    private final int limitSwitchPort;
    private final double percentOutput;

    public SetShoulderSpeed(Arm arm, int limitSwitchPort, double percentOutput) {
        this.arm = arm;
        this.limitSwitchPort = limitSwitchPort;
        this.percentOutput = percentOutput;
    }

    @Override
    public void initialize() {

        arm.SetBrakeMode(true);

        arm.SetPercentOutput(percentOutput);

        System.out.println("ENABLING ARM!!!!!!!!");
    }

    @Override
    public void execute() {


    }

    @Override
    public boolean isFinished() {

    return arm.limitSwitchOutput(limitSwitchPort);

    }

    @Override
    public void end(boolean interrupted) {

    arm.SetPercentOutput(0);
        System.out.println("DISABLING ARM!!!!!!!!");

    }

}

