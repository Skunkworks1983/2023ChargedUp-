package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

import static frc.robot.constants.Constants.Drivebase.VOLTAGE_TO_DISTANCE_SENSOR;


public class DetectRangeSensorCommand extends CommandBase {

    double distanceFront;

    double distanceBack;


    enum SensorDetectionMode {START,SEND_FREQUENCY,WAIT_FOR_FREQUENCY,WAIT_FOR_FREQUENCY_TWO,READ_FREQUENCY,WAIT_TO_SEND_FREQUENCY};

    SensorDetectionMode sensorMode = SensorDetectionMode.START;

    double frontVoltage;
    double backVoltage;

   Drivebase.DriveDirection currentDirection=Drivebase.DriveDirection.MOTIONLESS;

    Drivebase.DriveDirection directionToMessure= Drivebase.DriveDirection.FORWARD;

    public void setCurrentDirection(Drivebase.DriveDirection direction){currentDirection=direction;}

    public double getFrontVoltage(){return frontVoltage;}

    public double getBackVoltage(){return backVoltage;}
    public DetectRangeSensorCommand() {

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
        System.out.println("DetectRangeSensor started");
    }


    @Override
    public void execute() {

        completeState();
        }

    public void completeState(){

        switch (sensorMode){

            case START:
                directionToMessure=currentDirection;//Drivebase.GetDrivebase().getDriveDirection();
            sensorMode=SensorDetectionMode.SEND_FREQUENCY;
            break;

            case SEND_FREQUENCY:
                if (directionToMessure == Drivebase.DriveDirection.FORWARD) {
                    Drivebase.GetDrivebase().setFrontRangeSensor(true);
                }
                else{Drivebase.GetDrivebase().setBackRangeSensor(true);}
                sensorMode=SensorDetectionMode.WAIT_FOR_FREQUENCY;
                break;

            case WAIT_FOR_FREQUENCY:
                sensorMode=SensorDetectionMode.WAIT_FOR_FREQUENCY_TWO;
                break;
            case WAIT_FOR_FREQUENCY_TWO:

                sensorMode=SensorDetectionMode.READ_FREQUENCY;

                break;
            case READ_FREQUENCY:
                if (directionToMessure == Drivebase.DriveDirection.FORWARD) {
                    Drivebase.GetDrivebase().setFrontRangeVoltage(Drivebase.GetDrivebase().getFrontRangeSensor());}

                else {
                    Drivebase.GetDrivebase().setBackRangeVoltage(Drivebase.GetDrivebase().getBackRangeSensor());}
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


        System.out.println("DetectRangeSensor ended. Interupted:"+interrupted);
    }
}
