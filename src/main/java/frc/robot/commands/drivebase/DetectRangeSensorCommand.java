package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.multidrivebase.Drivebase4MotorTalonFX;

import static frc.robot.constants.Constants.Drivebase.VOLTAGE_TO_DISTANCE_SENSOR;


public class DetectRangeSensorCommand extends CommandBase {

    double distanceFront;

    double distanceBack;


    enum SensorDetectionMode {START,SEND_FREQUENCY,WAIT_FOR_FREQUENCY,WAIT_FOR_FREQUENCY_TWO,READ_FREQUENCY,WAIT_TO_SEND_FREQUENCY};

    SensorDetectionMode sensorMode = SensorDetectionMode.START;

    double frontVoltage;
    double backVoltage;

    Drivebase4MotorTalonFX.DriveDirection currentDirection=Drivebase4MotorTalonFX.DriveDirection.MOTIONLESS;

    Drivebase4MotorTalonFX.DriveDirection directionToMessure= Drivebase4MotorTalonFX.DriveDirection.FORWARD;

    public void setCurrentDirection(Drivebase4MotorTalonFX.DriveDirection direction){currentDirection=direction;}

    public double getFrontVoltage(){return frontVoltage;}

    public double getBackVoltage(){return backVoltage;}
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
        System.out.println("frontDistance: "+ frontVoltage +" backDistance: " + backVoltage);
    }

    public void completeState(){

        switch (sensorMode){

            case START:
                directionToMessure=currentDirection;//((Drivebase4MotorTalonFX)Drivebase4MotorTalonFX.GetDrivebase()).getDriveDirection();
            sensorMode=SensorDetectionMode.SEND_FREQUENCY;
            break;

            case SEND_FREQUENCY:
                if (directionToMessure == Drivebase4MotorTalonFX.DriveDirection.FORWARD) {
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
                if (directionToMessure == Drivebase4MotorTalonFX.DriveDirection.FORWARD) {
                    frontVoltage =((Drivebase4MotorTalonFX)Drivebase4MotorTalonFX.GetDrivebase()).getFrontRangeSensor();}

                else {
                    backVoltage = ((Drivebase4MotorTalonFX)Drivebase4MotorTalonFX.GetDrivebase()).getBackRangeSensor();}
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
