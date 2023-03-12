package frc.robot.commands.Collector;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;


public class HoldConeCommand extends CommandBase {
    private Collector collectorInstance;

    private int countConeHeld;
    private Arm armInstance = Arm.getInstance();

    public HoldConeCommand() {

        collectorInstance = Collector.getInstance();
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(collectorInstance);
    }

    @Override
    public void initialize() {
        System.out.println("Cone held command initialize");
        countConeHeld = 0;

    }

    @Override
    public void execute() {


            if (collectorInstance.coneCurrentHolding()) {
                countConeHeld++;
            } else {
                countConeHeld = 0;
            }
            if(countConeHeld >= Constants.Collector.CONE_COLLECTED_VALUE) {
                collectorInstance.Motor.set(TalonFXControlMode.PercentOutput, .01);
            }
            else{
                collectorInstance.Motor.set(TalonFXControlMode.PercentOutput, .0);
            }
    }

    @Override
    public boolean isFinished() {


        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        if(interrupted) {
            System.out.println("Hold cone command interrupted");
        }
        else{
            System.out.println("hold cone command end");
        }



        collectorInstance.Setspeed(0);
    }
}