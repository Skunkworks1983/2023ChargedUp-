package frc.robot.services;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.arm.RotateDegrees;
import frc.robot.commands.autos.CollectorExpelTeleopCommand;
import frc.robot.commands.autos.CollectorIntakeTeleopCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.multidrivebase.Drivebase;
import frc.robot.subsystems.Arm;


public class Oi
    {
        public static Oi Instance;
        Joystick leftStick;
        Joystick rightStick;

    JoystickButton armUpButton;
    JoystickButton armDownButton;


    JoystickButton intakeButton;
    JoystickButton expelButton;


    Arm arm = Arm.getInstance();

    public Oi(Drivebase drivebase) {
        System.out.println("oi init");
        Instance = this;
        leftStick = new Joystick(Constants.JoystickPorts.LEFT_JOY_STICK_PORT);
        rightStick = new Joystick(Constants.JoystickPorts.RIGHT_JOY_STICK_PORT);

        armUpButton = new JoystickButton(leftStick, Constants.OIButtons.ARM_UP_BUTTON);
        armDownButton = new JoystickButton(leftStick, Constants.OIButtons.ARM_DOWN_BUTTON);
        expelButton = new JoystickButton(leftStick, Constants.OIButtons.EXPEL_BUTTON);
        intakeButton = new JoystickButton(leftStick, Constants.OIButtons.INTAKE_BUTTON);

        //button sticks

        //when held
        expelButton.whileTrue(new CollectorExpelTeleopCommand());
        intakeButton.whileTrue(new CollectorIntakeTeleopCommand());

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