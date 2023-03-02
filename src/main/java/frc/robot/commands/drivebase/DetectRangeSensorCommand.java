package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.multidrivebase.Drivebase4MotorTalonFX;


public class DetectRangeSensorCommand extends CommandBase {

    double distanceFront;

    double distanceBack;


    enum SensorDetectionMode {START,SEND_FREQUENCY,WAIT_FOR_FREQUENCY,WAIT_FOR_FREQUENCY_TWO,READ_FREQUENCY,WAIT_TO_SEND_FREQUENCY};

    SensorDetectionMode sensorMode = SensorDetectionMode.START;

    double frontDistance;
    double backDistance;

    Drivebase4MotorTalonFX.DriveDirection currentDirection;
    public double getFrontDistance(){

        return frontDistance;

    }

    public double getBackDistance(){return backDistance;}
    public DetectRangeSensorCommand() {

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {


        ((Drivebase4MotorTalonFX)Drivebase4MotorTalonFX.GetDrivebase()).getFrontRangeSensor();

        ((Drivebase4MotorTalonFX)Drivebase4MotorTalonFX.GetDrivebase()).getBackRangeSensor();
        completeState();
        System.out.println("frontDistance: "+frontDistance+" backDistance: " +backDistance);
    }

    public void completeState(){

        switch (sensorMode){

            case START:
                currentDirection=((Drivebase4MotorTalonFX)Drivebase4MotorTalonFX.GetDrivebase()).getDriveDirection();
            sensorMode=SensorDetectionMode.SEND_FREQUENCY;
            break;

            case SEND_FREQUENCY:
                if (currentDirection == Drivebase4MotorTalonFX.DriveDirection.FORWARD) {
                    ((Drivebase4MotorTalonFX)Drivebase4MotorTalonFX.GetDrivebase()).setFrontRangeSensor(true);
                }
                else{((Drivebase4MotorTalonFX)Drivebase4MotorTalonFX.GetDrivebase()).setBackRangeSensor(true);}
                sensorMode=SensorDetectionMode.WAIT_FOR_FREQUENCY;
                break;

            case WAIT_FOR_FREQUENCY:
                sensorMode=SensorDetectionMode.WAIT_FOR_FREQUENCY_TWO;
                break;
            case WAIT_FOR_FREQUENCY_TWO:

                sensorMode=SensorDetectionMode.READ_FREQUENCY;

                break;
            case READ_FREQUENCY:
                if (currentDirection == Drivebase4MotorTalonFX.DriveDirection.FORWARD) {
                    frontDistance=((Drivebase4MotorTalonFX)Drivebase4MotorTalonFX.GetDrivebase()).getFrontRangeSensor();}

                else {backDistance= ((Drivebase4MotorTalonFX)Drivebase4MotorTalonFX.GetDrivebase()).getBackRangeSensor();}
                sensorMode=SensorDetectionMode.WAIT_TO_SEND_FREQUENCY;

                break;
            case WAIT_TO_SEND_FREQUENCY:

                sensorMode=SensorDetectionMode.START;

                break;


        }

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
