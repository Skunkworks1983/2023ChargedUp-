// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.autos.DriveOnChargeStationAndBalanceP2CommandGroup;
import frc.robot.commands.autos.ScoreAndExitCommunityP1CommandGroup;
import frc.robot.commands.autos.ScoreAndExitCommunityP2CommandGroup;
import frc.robot.commands.autos.SimpleAutoCommandGroup;
import frc.robot.commands.drivebase.ArcadeDrive;
import frc.robot.commands.drivebase.TankDrive;
import frc.robot.commands.Collector.IntakeConeCollectorCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.arm.WristRotateDegrees;
import frc.robot.commands.autos.PositionShoulderAndWrist;
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

    private SendableChooser autoChooser;
    private Drivebase drivebase = Drivebase.GetDrivebase();
    private Collector collector = Collector.getInstance();
    private Oi oi = new Oi(drivebase,collector);
    Command DriveOnChargeStationAndBalanceP2 = new DriveOnChargeStationAndBalanceP2CommandGroup();
    Command SimpleAuto = new SimpleAutoCommandGroup();
    Command ScoreAndExitCommunityP2 = new ScoreAndExitCommunityP2CommandGroup();
    Command ScoreAndExitCommunityP1 = new ScoreAndExitCommunityP1CommandGroup();
    Command scoreAndDriveOutP2 = new DriveOnChargeStationAndBalanceP2CommandGroup();
    private RobotContainer robotContainer;

    private Arm arm;

   autoChooser.addOption("SimpleAuto", new SimpleAutoCommandGroup());
        autoChooser.addOption("twoBallHighCenter", new TwoBallAutoCenter(theDrivebase, theCollector, theShooter));
        autoChooser.addOption("twoBallHighLeft", new TwoBallAutoLeft(theDrivebase, theCollector, theShooter));
        autoChooser.addOption("ExitTarmac", new ExitTarmac(theDrivebase));
        autoChooser.addOption("oneBallAutosHigh", new OneBallAutosHighCommandGroup(theShooter, theDrivebase));
        autoChooser.addOption("oneBallAutosLow", new OneBallAutosLowCommandGroup(theShooter, theDrivebase));
        SmartDashboard.putData("autoChooser", autoChooser);


    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.\
        SmartDashboard.putData("autoChooser", autoChooser);
        arm = Arm.getInstance();
        robotContainer = new RobotContainer();
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
        Drivebase.GetDrivebase().SetBrakeMode(false);
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
        CommandScheduler.getInstance().cancelAll();
        SendableChooser autoChooser = (SendableChooser) SmartDashboard.getData("autoChooser");
        autonomousCommand = (Command)autoChooser.getSelected();
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
        autoChooser.addOption();

     //  SendableChooser autoChooser = (SendableChooser) SmartDashboard.getData("autoChooser");
        DriveOnChargeStationAndBalanceP2.schedule();
        SimpleAuto.schedule();
        ScoreAndExitCommunityP2.schedule();
        ScoreAndExitCommunityP1.schedule();

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
