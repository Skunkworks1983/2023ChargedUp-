package frc.robot.services;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Collector.*;
import frc.robot.commands.arm.ResetArm;
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

        JoystickButton humanPlayerPickup;
        JoystickButton carry;

        JoystickButton scoreMid;

        JoystickButton intakeButton;
        JoystickButton expelButton;

        JoystickButton coneMode;

        JoystickButton cubeMode;
        JoystickButton manualToggle;

        JoystickButton floorPickup;

        JoystickButton resetArm;

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
        manualToggle = new JoystickButton(buttonStick,Constants.OIButtons.ENABLE_MANUAL);

        humanPlayerPickup = new JoystickButton(buttonStick, Constants.OIButtons.COLLECT_SHELF);
        carry = new JoystickButton(buttonStick, Constants.OIButtons.STOW);

        scoreMid = new JoystickButton(buttonStick, Constants.OIButtons.SCORE_MID);

        coneMode = new JoystickButton(buttonStick, Constants.OIButtons.CONE_MODE);

        cubeMode = new JoystickButton(buttonStick, Constants.OIButtons.CUBE_MODE);

        expelButton = new JoystickButton(buttonStick, Constants.OIButtons.EXPEL);
        intakeButton = new JoystickButton(buttonStick, Constants.OIButtons.INTAKE);

        floorWeirdScore = new JoystickButton(buttonStick, Constants.OIButtons.WRIST_UP);  /*   ???   */
        floorNormalScore = new JoystickButton(buttonStick, Constants.OIButtons.WRIST_DOWN); /*   ???   */ //floor cone

        floorPickup = new JoystickButton(buttonStick, Constants.OIButtons.COLLECT_GROUND);

        resetArm = new JoystickButton(buttonStick, );

        //when held
        expelButton.and(coneMode).whileTrue(new ExpelConeCommand());
        expelButton.and(cubeMode).whileTrue(new ExpelCubeCommand());
        intakeButton.and(coneMode).and(manualToggle).whileTrue(new IntakeConeManualCommand());
        intakeButton.and(cubeMode).and(manualToggle).whileTrue(new IntakeCubeManualCommand());
        intakeButton.and(coneMode).and(manualToggle.negate()).onTrue(new IntakeConeSmartCommand());
        intakeButton.and(cubeMode).and(manualToggle.negate()).onTrue(new IntakeCubeSmartCommand());

        floorWeirdScore.whileTrue(new SetArmPositionCommand(Constants.Arm.SHOULDER_RESTING_ANGLE, Constants.Arm.WRIST_RESTING_ANGLE));
        floorNormalScore.whileTrue(new SetArmPositionCommand(Constants.ArmPos.FLOOR_NORMAL_SCORE_SHOULDER, Constants.ArmPos.FLOOR_NORMAL_SCORE_WRIST));
        humanPlayerPickup.and(coneMode).whileTrue(new SetArmPositionCommand(Constants.ArmPos.PLAYER_CONE_PICKUP_SHOULDER, Constants.ArmPos.PLAYER_CONE_PICKUP_WRIST));
        carry.whileTrue(new SetArmPositionCommand(Constants.ArmPos.CARRY_SHOULDER, Constants.ArmPos.CARRY_WRIST));
        scoreMid.and(coneMode).whileTrue(new SetArmPositionCommand(Constants.ArmPos.SCORE_CONE_MID_SHOULDER, Constants.ArmPos.SCORE_CONE_MID_WRIST));
        // do we have a scoreMid.and(cubeMode) ?
        floorPickup.and(cubeMode).onTrue(new SetArmPositionCommand(Constants.ArmPos.FLOOR_CUBE_PICKUP_SHOULDER, Constants.ArmPos.FLOOR_CUBE_PICKUP_WRIST));
        floorPickup.and(coneMode).onTrue(new SetArmPositionCommand(Constants.ArmPos.CONE_FLOOR_PICKUP_SHOULDER, Constants.ArmPos.CONE_FLOOR_PICKUP_WRIST));
        resetArm.whileTrue(new ResetArm());
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