package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;


public class ExpelConeCommand extends CommandBase {
    private Collector collectorInstance;
    private Arm armInstance;
    private double timeout;
    private Timer timer = new Timer();


    public ExpelConeCommand() {
        this(-1);
    }
    public ExpelConeCommand(double timeout) {
        armInstance = Arm.getInstance();
        collectorInstance = Collector.getInstance();
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(collectorInstance);
        this.timeout = timeout;
    }

    @Override
    public void initialize() {

        if(armInstance.getShoulderAngle() < 0) {
            collectorInstance.Setspeed(Constants.Collector.EXPEL_MOTOR_SPEED);
        }
        else {
            collectorInstance.Setspeed(-Constants.Collector.EXPEL_MOTOR_SPEED);
        }
        timer.start();
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished() {
        return timeout != -1 && timer.hasElapsed(timeout);
    }

    @Override
    public void end(boolean interrupted) {
        collectorInstance.Setspeed(0);
    }
}
