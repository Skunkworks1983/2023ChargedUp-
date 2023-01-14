package frc.robot.services;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;


public class Oi
    {
        Joystick leftStick;
        Joystick rightStick;
        Joystick buttonStick;
        JoystickButton leftStickTrigger;
        JoystickButton collectorUpwardButton;
        JoystickButton collectorDownwardButton;
        JoystickButton loadBallsButton;
        JoystickButton manualMoveCollectorUp;
        JoystickButton manualMoveCollectorDown;
        JoystickButton indexerOutputButton;
        JoystickButton spinUpFlyWheelLowButton;
        JoystickButton spinUpFlyWheelHighButton;
        JoystickButton indexerManualShootButton;
        JoystickButton collectorIn;
        JoystickButton collectorOut;
        JoystickButton indexerShootWhenReady;
        JoystickButton enableManualControls;
        JoystickButton collectorEncoderReset;
        JoystickButton toggleClimber;

        public Oi(Drivebase drivebase)
        {

            leftStick = new Joystick(Constants.JoystickPorts.LEFT_JOY_STICK_PORT);
            rightStick = new Joystick(Constants.JoystickPorts.RIGHT_JOY_STICK_PORT);
            buttonStick = new Joystick(Constants.JoystickPorts.BUTTON_STICK_PORT);

            //button sticks

            //when held

            // when pressed
        }

        public double getLeftY()
        {
            return leftStick.getY();
        }

        public double getRightY()
        {
            return rightStick.getY();
        }

        public double getLeftX()
        {
            return leftStick.getX();
        }

        public double getRightX()
        {
            return rightStick.getX();
        }


    }