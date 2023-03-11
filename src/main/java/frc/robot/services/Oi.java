package frc.robot.services;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Collector.*;
import frc.robot.commands.arm.RotateWristByPowerCommand;
import frc.robot.commands.arm.SetShoulderSpeed;
import frc.robot.commands.autos.BalanceOnChargeStationCommand;
import frc.robot.commands.autos.DetectRangeSensorCommand;
import frc.robot.commands.drivebase.ArcadeDrive;
import frc.robot.commands.drivebase.TankDrive;
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

        JoystickButton intakeButton;
        JoystickButton expelButton;

        JoystickButton autoBalance;

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

        coneToggle = new JoystickButton(buttonStick, Constants.OIButtons.CONE_TOGGLE);

        expelButton = new JoystickButton(buttonStick, Constants.OIButtons.EXPEL_BUTTON);
        intakeButton = new JoystickButton(buttonStick, Constants.OIButtons.INTAKE_BUTTON);

        wristUp = new JoystickButton(buttonStick,Constants.OIButtons.WRIST_UP_BUTTON);//4
        wristDown = new JoystickButton(buttonStick,Constants.OIButtons.WRIST_DOWN_BUTTON);//2

        autoBalance = new JoystickButton(leftStick,Constants.OIButtons.AUTO_BALANCE);

        //when held

        DetectRangeSensorCommand a = new DetectRangeSensorCommand();
        BalanceOnChargeStationCommand b = new BalanceOnChargeStationCommand(.023,0,0,.08,a);
        autoBalance.whileFalse(new TankDrive(drivebase,this));

        expelButton.and(coneToggle).whileTrue(new ExpelConeCommand());
        expelButton.and(coneToggle.negate()).whileTrue(new ExpelCubeCommand());
        intakeButton.and(coneToggle).and(manualToggle).whileTrue(new IntakeConeManualCommand());
        intakeButton.and(coneToggle.negate()).and(manualToggle).whileTrue(new IntakeCubeManualCommand());
        intakeButton.and(coneToggle).and(manualToggle.negate()).onTrue(new IntakeConeSmartCommand());
        intakeButton.and(coneToggle.negate()).and(manualToggle.negate()).onTrue(new IntakeCubeSmartCommand());

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