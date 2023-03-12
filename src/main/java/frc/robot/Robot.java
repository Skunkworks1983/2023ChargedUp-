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
import frc.robot.commands.autos.DriveOnChargeStationAndBalanceP2ConeCommandGroup;
import frc.robot.commands.autos.DriveOnChargeStationAndBalanceP2CubeCommandGroup;
import frc.robot.commands.autos.E2ToGamePiece4;
import frc.robot.commands.autos.LeaveCommunityP2E2;
import frc.robot.commands.autos.ScoreAndDriveOutP3CommandGroup;
import frc.robot.commands.autos.ScoreAndExitCommunityP1CommandGroup;
import frc.robot.commands.autos.ScoreAndExitCommunityP2CommandGroup;
import frc.robot.commands.autos.SimpleAutoCommandGroup;
import frc.robot.commands.drivebase.ArcadeDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivebase;
import frc.robot.services.Oi;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{

    private Command autonomousCommand;
    private SendableChooser autoChooser;
    private Drivebase drivebase = Drivebase.GetDrivebase();
    private Collector collector = Collector.getInstance();
    private Oi oi = new Oi(drivebase,collector);
    Command DriveOnChargeStationAndBalanceP2 = new DriveOnChargeStationAndBalanceP2ConeCommandGroup();
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
    public void robotInit()
    {
        arm = Arm.getInstance();
        arm.WristMotor.setNeutralMode(NeutralMode.Coast);
        autoChooser = new SendableChooser();
        autoChooser.addOption("SimpleAuto", new SimpleAutoCommandGroup());
        autoChooser.addOption("DriveOnChargeStationAndBalanceConeP2", new DriveOnChargeStationAndBalanceP2ConeCommandGroup());
        autoChooser.addOption("ScoreAndExitCommunityP2", new ScoreAndExitCommunityP2CommandGroup());
        autoChooser.addOption("ScoreAndExitCommunityP1", new ScoreAndExitCommunityP1CommandGroup());
        autoChooser.addOption("E2toGamePiece4",new E2ToGamePiece4());
        autoChooser.addOption("LeaveCommunityP2E2",new LeaveCommunityP2E2());
        autoChooser.addOption("ScoreAndDriveOutP3",new ScoreAndDriveOutP3CommandGroup());
        autoChooser.addOption("DriveOnChargeStationAndBalanceCubeP2", new DriveOnChargeStationAndBalanceP2CubeCommandGroup());


        //autoChooser.addOption("oneBallAutosHigh", new OneBallAutosHighCommandGroup());
       // autoChooser.addOption("oneBallAutosLow", new OneBallAutosLowCommandGroup());
        SmartDashboard.putData("autoChooser", autoChooser);

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.\

        robotContainer = new RobotContainer();

        drivebase.waitForHeadingReliable();

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
    public void disabledInit()
    {
        drivebase.runMotor(0,0);
        arm.WristMotor.setNeutralMode(NeutralMode.Coast);
        drivebase.SetBrakeMode(true);

    }


    @Override
    public void disabledPeriodic() {
    }


    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit()
    {
        arm.WristMotor.setNeutralMode(NeutralMode.Brake);
        CommandScheduler.getInstance().cancelAll();
        SendableChooser autoChooser = (SendableChooser) SmartDashboard.getData("autoChooser");
        autonomousCommand = (Command)autoChooser.getSelected();
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
       // autoChooser.addOption();

     //  SendableChooser autoChooser = (SendableChooser) SmartDashboard.getData("autoChooser");
     //   DriveOnChargeStationAndBalanceP2.schedule();
     //   SimpleAuto.schedule();
     //   ScoreAndExitCommunityP2.schedule();
     //   ScoreAndExitCommunityP1.schedule();

        drivebase.waitForHeadingReliable();
        drivebase.SetBrakeMode(true);
    }


    @Override
    public void teleopInit()
    {
        drivebase.SetBrakeMode(true);
        Command arcadeDrive = new ArcadeDrive(drivebase, oi);
        arcadeDrive.schedule();

    }


    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}


    @Override
    public void testInit()
    {
        drivebase.SetBrakeMode(false);
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        arm = Arm.getInstance();
        arm.SetBrakeMode(false, arm.ShoulderMotor);
        arm.SetBrakeMode(false, arm.WristMotor);
    }


    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic()
    {
    }


    /** This method is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}


    /** This method is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
