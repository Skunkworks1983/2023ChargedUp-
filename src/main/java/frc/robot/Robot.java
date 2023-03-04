// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.arm.WaveCollectorCommandGroup;
import frc.robot.commands.autos.ScoreAndExitCommunityP2CommandGroup;
import frc.robot.commands.autos.SimpleAutoCommandGroup;
import frc.robot.commands.drivebase.TankDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.services.Oi;
import frc.robot.subsystems.Drivebase;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    private Drivebase drivebase = Drivebase.GetDrivebase();
    private Oi oi = new Oi(drivebase);
    Command SimpleAuto = new SimpleAutoCommandGroup();
    Command ScoreAndExitCommunityP2 = new ScoreAndExitCommunityP2CommandGroup();

    private RobotContainer robotContainer;

    private Arm arm;

    
    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
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
    public void disabledInit() {

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
      //  SimpleAuto.schedule();
        ScoreAndExitCommunityP2.schedule();
    }


    @Override
    public void teleopInit()
    {
        arm = Arm.getInstance();
        arm.ShoulderMotor.set(TalonFXControlMode.PercentOutput, 0);
        arm.ShoulderMotor.setNeutralMode(NeutralMode.Brake);

        //double rotateTo = 15;
        Command TankDrive = new TankDrive(drivebase, oi);

        TankDrive.schedule();

        /* if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        } */
    }
    
    
    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic()
    {
        arm = Arm.getInstance();

        System.out.println("Limit switch front: " + arm.limitSwitchOutput(Constants.Arm.SHOULDER_LIMIT_SWITCH_FRONT));
        System.out.println("Limit switch back: " + arm.limitSwitchOutput(Constants.Arm.SHOULDER_LIMIT_SWITCH_BACK));
    }
    
    
    /** This method is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}
    
    
    /** This method is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
