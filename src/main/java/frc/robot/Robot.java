// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.autos.CompAutos.*;
import frc.robot.commands.autos.*;
import frc.robot.commands.drivebase.ArcadeDrive;
import frc.robot.commands.drivebase.RotateWithEncoderCommand;
import frc.robot.constants.Constants;
import frc.robot.services.Oi;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.LimeLight;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private boolean setBrakeModeOnDisable = true;
    private Oi oi = Oi.GetInstance();
    private Command autonomousCommand;
    private SendableChooser autoChooser;
    private Drivebase drivebase = Drivebase.GetDrivebase();
    private Collector collector = Collector.getInstance();
    Command DriveOnChargeStationAndBalanceP2 = new ConeMidAndBalance4_6();
    Command SimpleAuto = new SimpleAutoCommandGroup();
    Command ScoreAndExitCommunityP2 = new ScoreAndExitCommunityP2CommandGroup();
    Command ScoreAndExitCommunityP1 = new ScoreAndExitCommunityP1CommandGroup();
    private RobotContainer robotContainer;

    private Arm arm;


    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        arm = Arm.getInstance();
        arm.WristMotor.setNeutralMode(NeutralMode.Coast);
        autoChooser = new SendableChooser();
        autoChooser.addOption("ConeMidAndBalance4_6", new ConeMidAndBalance4_6());
        autoChooser.addOption("CubeMidAndBalance5", new CubeMidAndBalance5());
        autoChooser.addOption("ConeMidLeaveCommunity1_9", new ConeMidLeaveCommunity1_9());
        autoChooser.addOption("CubeMidLeaveCommunity2_8", new CubeMidLeaveCommunity2_8());
        autoChooser.addOption("ConeLowAndBalance4_5_6", new ConeLowAndBalance4_5_6());
        autoChooser.addOption("CubeHighAndBalance5", new CubeHighAndBalance5());
        autoChooser.addOption("CubeHighLeaveCommunity2_8", new CubeHighLeaveCommunity2_8());
        autoChooser.addOption("DoNothing", new DoNothing());
//        autoChooser.addOption("TwoPieceBalance8Red", new TwoPieceBalance8Red());
//        autoChooser.addOption("TwoPieceBalance8Blue", new TwoPieceBalance8Blue());
//        autoChooser.addOption("TwoPieceBalance2Red", new TwoPieceBalance2Red());
//        autoChooser.addOption("TwoPieceBalance2Blue", new TwoPieceBalance2Blue());
//        autoChooser.addOption("TwoPiece8Red", new TwoPiece8Red());
//        autoChooser.addOption("TwoPiece8Blue", new TwoPiece8Blue());
//        autoChooser.addOption("TwoPiece2Red", new TwoPiece2Red());
//        autoChooser.addOption("TwoPiece2Blue", new TwoPiece2Blue());
        autoChooser.addOption("TrajectoryTwoPieceBumpRed", new TrajectoryTwoPieceBumpRed());
        autoChooser.addOption("TrajectoryTwoPieceBumpBlue", new TrajectoryTwoPieceBumpBlue());

        autoChooser.addOption("RotateWithEncoder", new RotateWithEncoderCommand(90));

        SmartDashboard.putData("autoChooser", autoChooser);

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.\

        robotContainer = new RobotContainer();

        drivebase.waitForHeadingReliable();
        drivebase.resetGyro();
        SmartDashboard.putNumber("floor cube pickup", Constants.ArmPos.FLOOR_CUBE_PICKUP_WRIST);
    }


    /**
     * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }


    /**
     * This method is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        drivebase.runMotor(0, 0);
        arm.WristMotor.setNeutralMode(NeutralMode.Coast);
        if (setBrakeModeOnDisable) {
            drivebase.SetBrakeMode(true);
        }
        arm.SetLightMode(Constants.Lights.PARTY);
        System.out.println("compete");
    }


    @Override
    public void disabledPeriodic() {
        SmartDashboard.putNumber("wrist angle: ", arm.getWristAngle());
    }


    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        //Drivebase.GetDrivebase().setPose(new Pose2d(Units.feetToMeters(5.9166), Units.feetToMeters(25.125), new Rotation2d(Math.PI)));

        Collector.getInstance().SetSpeed(0);
        arm.SetLightMode(Constants.Lights.BLANK);
        setBrakeModeOnDisable = true;
        arm.WristMotor.setNeutralMode(NeutralMode.Brake);//auto volocit kp /kd

        CommandScheduler.getInstance().cancelAll();
        SendableChooser autoChooser = (SendableChooser) SmartDashboard.getData("autoChooser");
        autonomousCommand = (Command) autoChooser.getSelected();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
            System.out.println("ENABLING AUTO");
        }
        else
        {
            System.out.println("auto is null");
        }
        LimeLight.getInstance().setEnable(true);
        //drivebase.waitForHeadingReliable();
        drivebase.SetBrakeMode(true);
    }


    @Override
    public void teleopInit() {
        arm.SetLightMode(Constants.Lights.BLANK);
        drivebase.setGyroStatus(false);
        setBrakeModeOnDisable = true;
        drivebase.SetBrakeMode(true);
        drivebase.setDefaultCommand(new ArcadeDrive(Drivebase.GetDrivebase(), Oi.GetInstance(),LimeLight.getInstance()));
    }


    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic()
{
    SmartDashboard.putData("field" , drivebase.getField());
}


    @Override
    public void testInit() {
        setBrakeModeOnDisable = false;
        drivebase.SetBrakeMode(false);
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        arm = Arm.getInstance();
        arm.SetBrakeMode(false, arm.ShoulderMotor);
        arm.SetBrakeMode(false, arm.WristMotor);
    }


    /**
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic()
    {
        System.out.println("shoulder back switch: " + arm.ShoulderMotor.getSensorCollection().isRevLimitSwitchClosed() + " wrist switch: " + arm.WristMotor.getSensorCollection().isRevLimitSwitchClosed());
    }


    /**
     * This method is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }


    /**
     * This method is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
