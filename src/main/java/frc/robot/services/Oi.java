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

import static frc.robot.constants.Constants.Collector.INTAKE_MOTOR_SPEED;


public class Oi
    {
        public static Oi Instance;
        Joystick leftStick;
        Joystick rightStick;
        Joystick buttonStick;

        JoystickButton floorNormalScore;

        JoystickButton humanPlayerPickup;
        JoystickButton carry;

        JoystickButton scoreMid;

        JoystickButton intakeButton;
        JoystickButton expelButton;

        JoystickButton coneToggle;
        JoystickButton manualToggle;

        JoystickButton manualShoulderUp;

        JoystickButton manualShoulderDown;

        JoystickButton shootCube;

        JoystickButton wristUp;

        JoystickButton wristDown;

        JoystickButton manualCollectorUp;

        JoystickButton manualCollectorDown;

        JoystickButton floorPickup;

        JoystickButton resetArm;

        JoystickButton collectorUp;

        JoystickButton collectorDown;

        JoystickButton shoulderUp;

        JoystickButton shoulderDown;

    public Oi()
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

        shootCube = new JoystickButton(buttonStick,Constants.OIButtons.SHOOT_CUBE);

        expelButton = new JoystickButton(buttonStick, Constants.OIButtons.EXPEL);
        intakeButton = new JoystickButton(buttonStick, Constants.OIButtons.INTAKE);

        wristUp = new JoystickButton(buttonStick,Constants.OIButtons.WRIST_UP);

        wristDown = new JoystickButton(buttonStick,Constants.OIButtons.WRIST_DOWN);

        manualCollectorUp = new JoystickButton(buttonStick,Constants.OIButtons.MANUAL_COLLECTOR_UP);

        manualCollectorDown = new JoystickButton(buttonStick,Constants.OIButtons.MANUAL_COLLECTOR_DOWN);

        manualShoulderUp= new JoystickButton(buttonStick,Constants.OIButtons.MANUAL_SHOULDER_UP);
        manualShoulderDown= new JoystickButton(buttonStick,Constants.OIButtons.MANUAL_SHOULDER_DOWN);

        floorNormalScore = new JoystickButton(buttonStick, Constants.OIButtons.SCORE_LOW);

        floorPickup = new JoystickButton(buttonStick, Constants.OIButtons.COLLECT_GROUND);

        resetArm = new JoystickButton(buttonStick, Constants.OIButtons.RESET_POSITION);

        //when held
        expelButton.and(coneMode).whileTrue(new ExpelConeCommand());
        expelButton.and(cubeMode).whileTrue(new ExpelCubeCommand());
        intakeButton.and(coneMode).and(manualToggle).whileTrue(new IntakeConeManualCommand());
        intakeButton.and(cubeMode).and(manualToggle).whileTrue(new IntakeCubeManualCommand());
        intakeButton.and(coneMode).and(manualToggle.negate()).onTrue(new IntakeConeSmartCommand());
        intakeButton.and(cubeMode).and(manualToggle.negate()).onTrue(new IntakeCubeSmartCommand());

        floorNormalScore.whileTrue(new SetArmPositionCommand(Constants.ArmPos.FLOOR_NORMAL_SCORE_SHOULDER, Constants.ArmPos.FLOOR_NORMAL_SCORE_WRIST));
        humanPlayerPickup.and(coneMode).whileTrue(new SetArmPositionCommand(Constants.ArmPos.PLAYER_CONE_PICKUP_SHOULDER, Constants.ArmPos.PLAYER_CONE_PICKUP_WRIST));
        carry.whileTrue(new SetArmPositionCommand(Constants.ArmPos.CARRY_SHOULDER, Constants.ArmPos.CARRY_WRIST));
        scoreMid.and(coneMode).whileTrue(new SetArmPositionCommand(Constants.ArmPos.SCORE_CONE_MID_SHOULDER, Constants.ArmPos.SCORE_CONE_MID_WRIST));
        scoreMid.and(cubeMode).whileTrue(new SetArmPositionCommand(Constants.ArmPos.SCORE_CUBE_MID_SHOULDER, Constants.ArmPos.SCORE_CUBE_MID_WRIST));
        floorPickup.and(cubeMode).onTrue(new SetArmPositionCommand(Constants.ArmPos.FLOOR_CUBE_PICKUP_SHOULDER, Constants.ArmPos.FLOOR_CUBE_PICKUP_WRIST));
        floorPickup.and(coneMode).onTrue(new SetArmPositionCommand(Constants.ArmPos.CONE_FLOOR_PICKUP_SHOULDER, Constants.ArmPos.CONE_FLOOR_PICKUP_WRIST));
        resetArm.whileTrue(new ResetArm());
        wristUp.whileTrue(new RotateWristByPowerCommand(Constants.Arm.WRIST_POWER));
        wristDown.whileTrue(new RotateWristByPowerCommand(-Constants.Arm.WRIST_POWER));
        shoulderUp.whileTrue(new SetShoulderSpeed(Arm.getInstance(),Constants.Arm.SHOULDER_LIMIT_SWITCH_FRONT,Constants.Arm.SHOULDER_MANUAL_SPEED));
        shoulderDown.whileTrue(new SetShoulderSpeed(Arm.getInstance(),Constants.Arm.SHOULDER_LIMIT_SWITCH_FRONT,-Constants.Arm.SHOULDER_MANUAL_SPEED));
        collectorDown.whileTrue(new CollectorPercentOutputCommand(.1));
        collectorUp.whileTrue(new CollectorPercentOutputCommand(-.1));

        floorNormalScore.whileTrue(new SetArmPositionCommand(Constants.ArmPos.FLOOR_NORMAL_SCORE_SHOULDER, Constants.ArmPos.FLOOR_NORMAL_SCORE_WRIST));
        shootCube.onTrue(new SetArmPositionCommand
                (Constants.ArmPos.SCORE_CUBE_HIGH_SHOULDER, Constants.ArmPos.SCORE_CUBE_HIGH_WRIST));
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