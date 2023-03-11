package frc.robot.services;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Collector.*;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class Oi
    {
        public static Oi Instance;
        Joystick leftStick;
        Joystick rightStick;
        Joystick buttonStick;

        JoystickButton floorWeirdScore;
        JoystickButton floorNormalScore;

        JoystickButton humanPlayerConePickup;
        JoystickButton carry;

        JoystickButton scoreMid;

        JoystickButton intakeButton;
        JoystickButton expelButton;

        JoystickButton coneToggle;
        JoystickButton manualToggle;

        JoystickButton coneFloorPickup;
        JoystickButton cubeFloorPickup;

        Arm arm = Arm.getInstance();

        Collector collector = Collector.getInstance();




    public Oi(Drivebase drivebase, Collector collector)
    {



        System.out.println("oi init");
        Instance = this;
        leftStick = new Joystick(Constants.JoystickPorts.LEFT_JOY_STICK_PORT);
        rightStick = new Joystick(Constants.JoystickPorts.RIGHT_JOY_STICK_PORT);
        buttonStick = new Joystick(Constants.JoystickPorts.BUTTON_STICK_PORT);

        //button sticks
        manualToggle = new JoystickButton(buttonStick,Constants.OIButtons.MANUAL_TOGGLE);

        humanPlayerConePickup = new JoystickButton(buttonStick, Constants.OIButtons.ARM_UP_BUTTON);
        carry = new JoystickButton(buttonStick, Constants.OIButtons.ARM_DOWN_BUTTON);

        scoreMid = new JoystickButton(buttonStick, Constants.OIButtons.SCORE_MID);

        coneToggle = new JoystickButton(buttonStick, Constants.OIButtons.CONE_TOGGLE);

        expelButton = new JoystickButton(buttonStick, Constants.OIButtons.EXPEL_BUTTON);
        intakeButton = new JoystickButton(buttonStick, Constants.OIButtons.INTAKE_BUTTON);

        floorWeirdScore = new JoystickButton(buttonStick, Constants.OIButtons.WRIST_UP_BUTTON);//4
        floorNormalScore = new JoystickButton(buttonStick, Constants.OIButtons.WRIST_DOWN_BUTTON);//2

        coneFloorPickup = new JoystickButton(buttonStick, Constants.OIButtons.CONE_FLOOR_PICKUP);
        cubeFloorPickup = new JoystickButton(buttonStick, 3);

        //when held
        expelButton.and(coneToggle).whileTrue(new ExpelConeCommand());
        expelButton.and(coneToggle.negate()).whileTrue(new ExpelCubeCommand());
        intakeButton.and(coneToggle).and(manualToggle).whileTrue(new IntakeConeManualCommand());
        intakeButton.and(coneToggle.negate()).and(manualToggle).whileTrue(new IntakeCubeManualCommand());
        intakeButton.and(coneToggle).and(manualToggle.negate()).onTrue(new IntakeConeAndHoldCommandGroup());
        intakeButton.and(coneToggle.negate()).and(manualToggle.negate()).onTrue(new IntakeCubeSmartCommand());

        floorWeirdScore.whileTrue(new SetArmPositionCommand(Constants.Arm.SHOULDER_RESTING_ANGLE, Constants.Arm.WRIST_RESTING_ANGLE));
        floorNormalScore.whileTrue(new SetArmPositionCommand(Constants.ArmPos.FLOOR_NORMAL_SCORE_SHOULDER, Constants.ArmPos.FLOOR_NORMAL_SCORE_WRIST));
        humanPlayerConePickup.onTrue(new SetArmPositionCommand(Constants.ArmPos.PLAYER_CONE_PICKUP_SHOULDER, Constants.ArmPos.PLAYER_CONE_PICKUP_WRIST));
        carry.whileTrue(new SetArmPositionCommand(Constants.ArmPos.CARRY_SHOULDER, Constants.ArmPos.CARRY_WRIST));
        scoreMid.whileTrue(new SetArmPositionCommand(Constants.ArmPos.SCORE_CONE_MID_SHOULDER, Constants.ArmPos.SCORE_CONE_MID_WRIST));
        cubeFloorPickup.onTrue(new SetArmPositionCommand(Constants.ArmPos.FLOOR_CUBE_PICKUP_SHOULDER, Constants.ArmPos.FLOOR_CUBE_PICKUP_WRIST));
        coneFloorPickup.onTrue(new SetArmPositionCommand(Constants.ArmPos.CONE_FLOOR_PICKUP_SHOULDER, Constants.ArmPos.CONE_FLOOR_PICKUP_WRIST));
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