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

    JoystickButton coneToggle;
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
        coneToggle = new JoystickButton(buttonStick, Constants.OIButtons.CONE_TOGGLE);

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
        expelButton.and(coneToggle).whileTrue(new ExpelConeCommand());
        expelButton.and(coneToggle.negate()).whileTrue(new ExpelCubeCommand());
        intakeButton.and(coneToggle).and(manualToggle).whileTrue(new IntakeConeManualCommand());
        intakeButton.and(coneToggle.negate()).and(manualToggle).whileTrue(new IntakeCubeManualCommand());
        intakeButton.and(coneToggle).and(manualToggle.negate()).whileTrue(new IntakeConeAndStowCommand());
        intakeButton.and(coneToggle.negate()).and(manualToggle.negate()).whileTrue(new IntakeCubeAndStowCommand());

        floorNormalScore.whileTrue(new SetArmPositionCommand(Constants.ArmPose.FLOOR_NORMAL));
        scoreWeird.and(coneToggle.negate()).whileTrue(new SetArmPositionCommand(Constants.ArmPose.HIGH_CUBE));
        scoreWeird.and(coneToggle).whileTrue(new SetArmPositionCommand(Constants.ArmPose.FLOOR_WEIRD));
        humanPlayerPickup.and(coneToggle.negate()).whileTrue(new SetArmPositionCommand(Constants.ArmPose.SUBSTATION_CUBE));
        humanPlayerPickup.and(coneToggle).whileTrue(new SetArmPositionCommand(Constants.ArmPose.SUBSTATION_CONE));
        carry.onTrue(new SetArmPositionCommand(Constants.ArmPose.STOW));
        scoreMid.and(coneToggle.negate()).whileTrue(new SetArmPositionCommand(Constants.ArmPose.SCORE_MID_CUBE));
        scoreMid.and(coneToggle).whileTrue(new ScoreMidConeAndHoldCommandGroup());
        floorPickup.and(coneToggle.negate()).whileTrue(new SetArmPositionCommand(Constants.ArmPose.FLOOR_CUBE));
        floorPickup.and(coneToggle).whileTrue(new SetArmPositionCommand(Constants.ArmPose.FLOOR_CONE));

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
