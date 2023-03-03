package frc.robot.services;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.arm.RotateWristByPowerCommand;
import frc.robot.commands.arm.SetShoulderSpeed;
import frc.robot.commands.Collector.CollectorExpelTeleopCommand;
import frc.robot.commands.Collector.CollectorIntakeTeleopCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.drivebase.Drivebase4MotorTalonFX;


public class Oi
    {
        public static Oi Instance;
        Joystick leftStick;
        Joystick rightStick;
        Joystick buttonStick;

        JoystickButton wristUp;
        JoystickButton wristDown;

        JoystickButton armUpButton;
        JoystickButton armDownButton;

        JoystickButton intakeButton;
        JoystickButton expelButton;

        Arm arm = Arm.getInstance();

    public Oi(Drivebase4MotorTalonFX drivebase)
    {
        System.out.println("oi init");
        Instance = this;
        leftStick = new Joystick(Constants.JoystickPorts.LEFT_JOY_STICK_PORT);
        rightStick = new Joystick(Constants.JoystickPorts.RIGHT_JOY_STICK_PORT);
        buttonStick = new Joystick(Constants.JoystickPorts.BUTTON_STICK_PORT);

        //button sticks
        armUpButton = new JoystickButton(buttonStick, Constants.OIButtons.ARM_UP_BUTTON);
        armDownButton = new JoystickButton(buttonStick, Constants.OIButtons.ARM_DOWN_BUTTON);
        expelButton = new JoystickButton(buttonStick, Constants.OIButtons.EXPEL_BUTTON);
        intakeButton = new JoystickButton(buttonStick, Constants.OIButtons.INTAKE_BUTTON);

        wristUp = new JoystickButton(buttonStick,Constants.OIButtons.WRIST_UP_BUTTON);//4
        wristDown = new JoystickButton(buttonStick,Constants.OIButtons.WRIST_DOWN_BUTTON);//2

        //when held
        expelButton.whileTrue(new CollectorExpelTeleopCommand());
        intakeButton.whileTrue(new CollectorIntakeTeleopCommand());
        wristUp.whileTrue(new RotateWristByPowerCommand(-.11));
        wristDown.whileTrue(new RotateWristByPowerCommand(.11));
        Arm arm = Arm.getInstance();
            armUpButton.whileTrue(new SetShoulderSpeed(arm, Constants.Arm.SHOULDER_LIMIT_SWITCH_FRONT, 0.08));
        armDownButton.whileTrue(new SetShoulderSpeed(arm, Constants.Arm.SHOULDER_LIMIT_SWITCH_BACK, -0.04));
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