package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.multidrivebase.Drivebase4MotorTalonFX;

import static frc.robot.constants.Constants.Drivebase.VOLTAGE_TO_DISTANCE_SENSOR;


public class DetectRangeSensorWithoutDrivebaseCommand extends CommandBase {

    double distanceFront;

    double distanceBack;


    enum SensorDetectionMode {START,SEND_FREQUENCY,WAIT_FOR_FREQUENCY,WAIT_FOR_FREQUENCY_TWO,READ_FREQUENCY,WAIT_TO_SEND_FREQUENCY};

    SensorDetectionMode sensorMode = SensorDetectionMode.START;

    double frontVoltage;
    double backVoltage;

    DigitalOutput frontRangeSensorTrigger = new DigitalOutput(Constants.Drivebase.FRONT_RANGE_SENSOR_OUTPUT_CHANNEL);

    AnalogInput frontRangeSensorValue = new AnalogInput(Constants.Drivebase.FRONT_RANGE_SENSOR_INPUT_CHANNEL);



    Drivebase4MotorTalonFX.DriveDirection currentDirection;
    public double getBackDistance(){return backVoltage*VOLTAGE_TO_DISTANCE_SENSOR;}
    public double getFrontDistance(){return frontVoltage*VOLTAGE_TO_DISTANCE_SENSOR;}

    public double getFrontVoltage(){return frontVoltage;}

    public double getBackVoltage(){return backVoltage;}

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        completeState();
        System.out.println("frontDistance: "+ frontVoltage +" backDistance: " + backVoltage);
    }

    public void completeState(){

        switch (sensorMode){

            case START:
                sensorMode=SensorDetectionMode.SEND_FREQUENCY;
                break;

            case SEND_FREQUENCY:

                    frontRangeSensorTrigger.set(true);

                sensorMode=SensorDetectionMode.WAIT_FOR_FREQUENCY;
                break;

            case WAIT_FOR_FREQUENCY:
                sensorMode=SensorDetectionMode.WAIT_FOR_FREQUENCY_TWO;
                break;
            case WAIT_FOR_FREQUENCY_TWO:

                sensorMode=SensorDetectionMode.READ_FREQUENCY;

                break;
            case READ_FREQUENCY:

                    frontVoltage =frontRangeSensorValue.getValue();
                    sensorMode=SensorDetectionMode.WAIT_TO_SEND_FREQUENCY;
                System.out.println("front distance: "+ frontVoltage);

                break;
            case WAIT_TO_SEND_FREQUENCY:
                frontRangeSensorTrigger.set(false);//double check where this goes
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
