package frc.robot.services;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.arm.RotateWristByPowerCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;


public class Oi
    {
        Joystick leftStick;
        Joystick rightStick;

        Joystick buttonStick;

        JoystickButton wristUp;

        JoystickButton wristDown;

        public Oi(Drivebase drivebase)
        {

            leftStick = new Joystick(Constants.JoystickPorts.LEFT_JOY_STICK_PORT);
            rightStick = new Joystick(Constants.JoystickPorts.RIGHT_JOY_STICK_PORT);
            buttonStick = new Joystick(Constants.JoystickPorts.BUTTON_STICK_PORT);



            //button sticks
            wristUp = new JoystickButton(buttonStick,4);
            wristDown = new JoystickButton(buttonStick,2);
            //when held

            wristUp.whileTrue(new RotateWristByPowerCommand(-.09));
            wristDown.whileTrue(new RotateWristByPowerCommand(.09));

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