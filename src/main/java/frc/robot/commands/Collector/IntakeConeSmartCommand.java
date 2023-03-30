package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivebase;


public class IntakeConeSmartCommand extends CommandBase {
    private final Collector collectorInstance;
    private final Drivebase drivebaseInstance = Drivebase.GetDrivebase();
    private final Arm armInstance;
    private int countConeHeld;
    private int ticksElapsed;



    public IntakeConeSmartCommand() {
        armInstance = Arm.getInstance();
        collectorInstance = Collector.getInstance();

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(collectorInstance);
    }

    @Override
    public void initialize() {

        System.out.println("Intake Cone Smart Command Initialize");
        countConeHeld = 0;
        ticksElapsed = 0;

    }


    @Override
    public void execute() {

        double Speed =
                Constants.Collector.SCALE_BY_DRIVEBASE * drivebaseInstance.currentMotorSpeed * Constants.Collector.INTAKE_MOTOR_SPEED
                        + Constants.Collector.INTAKE_MOTOR_SPEED;
        SmartDashboard.putNumber("collector speed", Speed);
        //Collector Velocity=Y = Mx + B
        collectorInstance.SetSpeed(armInstance.getCurrentPose().ConeIntake() * Speed);


        if (collectorInstance.isHoldingCone()) {
            countConeHeld++;
        } else {
            countConeHeld = 0;
        }
        ticksElapsed++;
    }

    @Override
    public boolean isFinished() {
        return countConeHeld >= Constants.Collector.CONE_COLLECTED_VALUE && ticksElapsed >= Constants.Collector.TICKS_BEFORE_FINISHED;
    }

    @Override
    public void end(boolean interrupted) {
        collectorInstance.SetSpeed(0);
        if (interrupted) {
            System.out.println("Intake Cone Smart Command Ended, interrupted");
        } else {
            System.out.println("Intake Cone Smart Command Ended");
        }
    }
}
