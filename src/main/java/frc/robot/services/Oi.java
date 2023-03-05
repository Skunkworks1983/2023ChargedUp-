package frc.robot.services;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Collector.*;
import frc.robot.commands.arm.RotateWristByPowerCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.arm.SetShoulderSpeed;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;


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

        JoystickButton manualIntake;

        JoystickButton intakeButton;
        JoystickButton expelButton;

        JoystickButton coneToggle;
        JoystickButton manualToggle;

        Arm arm = Arm.getInstance();

    public Oi(Drivebase drivebase)
    {
        System.out.println("oi init");
        Instance = this;
        leftStick = new Joystick(Constants.JoystickPorts.LEFT_JOY_STICK_PORT);
        rightStick = new Joystick(Constants.JoystickPorts.RIGHT_JOY_STICK_PORT);
        buttonStick = new Joystick(Constants.JoystickPorts.BUTTON_STICK_PORT);

        //button sticks
        manualToggle = new JoystickButton(buttonStick,Constants.OIButtons.MANUAL_TOGGLE);

        armUpButton = new JoystickButton(buttonStick, Constants.OIButtons.ARM_UP_BUTTON);
        armDownButton = new JoystickButton(buttonStick, Constants.OIButtons.ARM_DOWN_BUTTON);

        manualIntake = new JoystickButton(buttonStick, 14);

        coneToggle = new JoystickButton(buttonStick, Constants.OIButtons.CONE_TOGGLE);

        expelButton = new JoystickButton(buttonStick, Constants.OIButtons.EXPEL_BUTTON);
        intakeButton = new JoystickButton(buttonStick, Constants.OIButtons.INTAKE_BUTTON);

        wristUp = new JoystickButton(buttonStick,Constants.OIButtons.WRIST_UP_BUTTON);//4
        wristDown = new JoystickButton(buttonStick,Constants.OIButtons.WRIST_DOWN_BUTTON);//2

        //when held
        expelButton.and(coneToggle).whileTrue(new ExpelConeCommand());
        expelButton.and(coneToggle.negate()).whileTrue(new ExpelCubeCommand());
        intakeButton.and(coneToggle).and(manualToggle).whileTrue(new IntakeConeManualCommand());
        intakeButton.and(coneToggle.negate()).and(manualToggle).whileTrue(new IntakeCubeManualCommand());
        intakeButton.and(coneToggle).and(manualToggle.negate()).onTrue(new IntakeConeSmartCommand());
        intakeButton.and(coneToggle.negate()).and(manualToggle.negate()).onTrue(new IntakeCubeSmartCommand());

        wristUp.whileTrue(new RotateWristByPowerCommand(-.11));
        wristDown.whileTrue(new RotateWristByPowerCommand(.11));
        armUpButton.whileTrue(new SetArmPositionCommand(Constants.ArmPos.PLAYER_CONE_PICKUP_SHOULDER, Constants.ArmPos.PLAYER_CONE_PICKUP_WRIST));
        armDownButton.whileTrue(new SetArmPositionCommand(Constants.Arm.SHOULDER_RESTING_ANGLE, 15.531));
        manualIntake.whileTrue(new SetArmPositionCommand(Constants.ArmPos.SCORE_CONE_MID_SHOULDER, Constants.ArmPos.SCORE_CONE_MID_WRIST));
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