package frc.robot.services;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Collector.*;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.arm.RotateWristByPowerCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.arm.SetLightsCommand;
import frc.robot.commands.arm.SetShoulderSpeed;
import frc.robot.commands.autos.SafeBalanceCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

public class Oi {

    public static Oi Instance;

    Joystick leftStick;
    Joystick rightStick;
    Joystick buttonStick;

    JoystickButton slowMode;

    JoystickButton centerOnPiece;

    JoystickButton floorNormalScore;

    JoystickButton humanPlayerPickup;
    JoystickButton carry;

    JoystickButton scoreMid;

    JoystickButton intakeButton;
    JoystickButton expelButton;

    JoystickButton cubeToggle;
    JoystickButton manualToggle;

    JoystickButton manualShoulderUp;

    JoystickButton manualShoulderDown;

    JoystickButton scoreWeird;

    JoystickButton wristUp;

    JoystickButton wristDown;

    JoystickButton manualCollectorUp;

    JoystickButton manualCollectorDown;

    JoystickButton floorPickup;

    JoystickButton resetArm;

    JoystickButton lightSwitchCube;
    JoystickButton lightSwitchCone;
    JoystickButton funSwitch1;
    JoystickButton funSwitch2;

    JoystickButton balanceButton;

    private Oi() {

        System.out.println("oi init");

        Instance = this;
        leftStick = new Joystick(Constants.JoystickPorts.LEFT_JOY_STICK_PORT);
        rightStick = new Joystick(Constants.JoystickPorts.RIGHT_JOY_STICK_PORT);
        buttonStick = new Joystick(Constants.JoystickPorts.BUTTON_STICK_PORT);

        //button sticks
        manualToggle = new JoystickButton(buttonStick, Constants.OIButtons.ENABLE_MANUAL);

        humanPlayerPickup = new JoystickButton(buttonStick, Constants.OIButtons.COLLECT_SHELF);
        carry = new JoystickButton(buttonStick, Constants.OIButtons.STOW);

        scoreMid = new JoystickButton(buttonStick, Constants.OIButtons.SCORE_MID);
        balanceButton = new JoystickButton(leftStick,3);//make constant later
        cubeToggle = new JoystickButton(buttonStick, Constants.OIButtons.CONE_TOGGLE);

        scoreWeird = new JoystickButton(buttonStick, Constants.OIButtons.SHOOT_CUBE);

        expelButton = new JoystickButton(buttonStick, Constants.OIButtons.EXPEL);
        intakeButton = new JoystickButton(buttonStick, Constants.OIButtons.INTAKE);

        wristUp = new JoystickButton(buttonStick, Constants.OIButtons.WRIST_UP);

        wristDown = new JoystickButton(buttonStick, Constants.OIButtons.WRIST_DOWN);

        manualCollectorUp = new JoystickButton(buttonStick, Constants.OIButtons.MANUAL_COLLECTOR_UP);

        manualCollectorDown = new JoystickButton(buttonStick, Constants.OIButtons.MANUAL_COLLECTOR_DOWN);

        manualShoulderUp = new JoystickButton(buttonStick, Constants.OIButtons.MANUAL_SHOULDER_UP);
        manualShoulderDown = new JoystickButton(buttonStick, Constants.OIButtons.MANUAL_SHOULDER_DOWN);

        floorNormalScore = new JoystickButton(buttonStick, Constants.OIButtons.SCORE_LOW);
        floorPickup = new JoystickButton(buttonStick, Constants.OIButtons.COLLECT_GROUND);

        resetArm = new JoystickButton(buttonStick, Constants.OIButtons.RESET_POSITION);

        slowMode = new JoystickButton(rightStick, Constants.OIButtons.DRIVE_SLOW);

        centerOnPiece = new JoystickButton(leftStick, Constants.OIButtons.CENTER_ON_PIECE);

        lightSwitchCube = new JoystickButton(buttonStick, 16);
        lightSwitchCone = new JoystickButton(buttonStick, 9);
        funSwitch1 = new JoystickButton(buttonStick, 19);
        funSwitch2 = new JoystickButton(buttonStick, 20);

        //when held
        expelButton.and((cubeToggle).negate()).whileTrue(new ExpelConeCommand());
        expelButton.and(cubeToggle).whileTrue(new ExpelCubeCommand());
        intakeButton.and((cubeToggle).negate()).and(manualToggle).whileTrue(new IntakeConeManualCommand());
        intakeButton.and(cubeToggle).and(manualToggle).whileTrue(new IntakeCubeManualCommand());
        intakeButton.and((cubeToggle).negate()).and(manualToggle.negate()).whileTrue(new IntakeConeSmartCommand());
        intakeButton.and(cubeToggle).and(manualToggle.negate()).whileTrue(new IntakeCubeSmartCommand());

        floorNormalScore.whileTrue(new SetArmPositionCommand(Constants.ArmPos.FLOOR_NORMAL_SCORE_SHOULDER, Constants.ArmPos.FLOOR_NORMAL_SCORE_WRIST));
        humanPlayerPickup.and(cubeToggle.negate()).whileTrue(new SetArmPositionCommand(Constants.ArmPos.PLAYER_CONE_PICKUP_SHOULDER, Constants.ArmPos.PLAYER_CONE_PICKUP_WRIST));
        humanPlayerPickup.and(cubeToggle).whileTrue(new SetArmPositionCommand(Constants.ArmPos.PLAYER_CUBE_PICKUP_SHOULDER, Constants.ArmPos.PLAYER_CUBE_PICKUP_WRIST));

        carry.onTrue(new SetArmPositionCommand(Constants.ArmPos.CARRY_SHOULDER, Constants.ArmPos.CARRY_WRIST));
        scoreMid.and(cubeToggle.negate()).whileTrue(new SetArmPositionCommand(Constants.ArmPos.SCORE_CONE_MID_SHOULDER, Constants.ArmPos.SCORE_CONE_MID_WRIST));
        scoreMid.and(cubeToggle).whileTrue(new SetArmPositionCommand(Constants.ArmPos.SCORE_CUBE_MID_SHOULDER, Constants.ArmPos.SCORE_CUBE_MID_WRIST));
        floorPickup.and(cubeToggle).onTrue(new SetArmPositionCommand(Constants.ArmPos.FLOOR_CUBE_PICKUP_SHOULDER, Constants.ArmPos.FLOOR_CUBE_PICKUP_WRIST));
        floorPickup.and(cubeToggle.negate()).onTrue(new SetArmPositionCommand(Constants.ArmPos.CONE_FLOOR_PICKUP_SHOULDER, Constants.ArmPos.CONE_FLOOR_PICKUP_WRIST));
        (resetArm.negate()).whileTrue(new ResetArm());
        wristUp.whileTrue(new RotateWristByPowerCommand(Constants.Arm.WRIST_POWER));
        wristDown.whileTrue(new RotateWristByPowerCommand(-Constants.Arm.WRIST_POWER));
        manualShoulderUp.whileTrue(new SetShoulderSpeed(Arm.getInstance(), Constants.Arm.SHOULDER_LIMIT_SWITCH_FRONT, Constants.Arm.SHOULDER_MANUAL_SPEED));
        manualShoulderDown.whileTrue(new SetShoulderSpeed(Arm.getInstance(), Constants.Arm.SHOULDER_LIMIT_SWITCH_FRONT, -Constants.Arm.SHOULDER_MANUAL_SPEED));
        manualCollectorUp.whileTrue(new CollectorPercentOutputCommand(.1));
        manualCollectorDown.whileTrue(new CollectorPercentOutputCommand(-.1));
        lightSwitchCube.whileTrue(new SetLightsCommand(Constants.Lights.CUBE));
        lightSwitchCone.whileTrue(new SetLightsCommand(Constants.Lights.CONE));
        funSwitch1.whileTrue(new SetLightsCommand(Constants.Lights.CYLON));
        funSwitch1.whileTrue(new SetLightsCommand(Constants.Lights.PARTY));
        balanceButton.whileTrue(new SafeBalanceCommandGroup());
        //manualToggle.onTrue(new ChangeGyroStatus(false));


        floorNormalScore.whileTrue(new SetArmPositionCommand(Constants.ArmPos.FLOOR_NORMAL_SCORE_SHOULDER, Constants.ArmPos.FLOOR_NORMAL_SCORE_WRIST));
        scoreWeird.and(cubeToggle).onTrue(new SetArmPositionCommand
                (Constants.ArmPos.SCORE_CUBE_HIGH_SHOULDER, Constants.ArmPos.SCORE_CUBE_HIGH_WRIST));
        scoreWeird.and(cubeToggle.negate()).onTrue(new SetArmPositionCommand
                (Constants.ArmPos.SCORE_CONE_WEIRD_SHOULDER, Constants.ArmPos.SCORE_CODE_WEIRD_WRIST));
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

    public boolean isSlowMode () {

        return slowMode.getAsBoolean();
    }

    public boolean isCenterOnPiece () {

        return centerOnPiece.getAsBoolean();
    }

    public static Oi GetInstance() {

        if (Instance == null) {
            Instance = new Oi();
        }

        return Instance;

    }
}