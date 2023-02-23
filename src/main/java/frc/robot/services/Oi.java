package frc.robot.services;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.arm.RotateDegrees;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;


public class Oi {
    Joystick leftStick;
    Joystick rightStick;

    JoystickButton armUpButton;
    JoystickButton armDownButton;

    Arm arm = Arm.getInstance();

    public Oi(Drivebase drivebase) {
        System.out.println("oi init");

        leftStick = new Joystick(Constants.JoystickPorts.LEFT_JOY_STICK_PORT);
        rightStick = new Joystick(Constants.JoystickPorts.RIGHT_JOY_STICK_PORT);

        armUpButton = new JoystickButton(leftStick, Constants.OIButtons.ARM_UP_BUTTON);
        armDownButton = new JoystickButton(leftStick, Constants.OIButtons.ARM_DOWN_BUTTON);

        //button sticks

        //when held

        // when pressed
        armUpButton.onTrue(new RotateDegrees(arm, 45, true));
        armDownButton.onTrue(new RotateDegrees(arm, -45, true));
    }

    public double getLeftY() {
        return leftStick.getY();
    }

    public double getRightY() {
        return rightStick.getY();
    }

    public double getLeftX() {
        return leftStick.getX();
    }

    public double getRightX() {
        return rightStick.getX();
    }
}