// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.arm.RotateDegrees;
import frc.robot.commands.autos.WaveCollectorCommandGroup;
import frc.robot.commands.autos.CollectorIntakeAutoCommand;
import frc.robot.commands.autos.CollectorTestingCommand;
import frc.robot.commands.drivebase.ArcadeDrive;
import frc.robot.commands.drivebase.TankDrive;
import frc.robot.subsystems.Arm;
import frc.robot.commands.drivebase.DriveDistanceCommand;
import frc.robot.services.Oi;
import frc.robot.subsystems.multidrivebase.Drivebase;
import frc.robot.subsystems.multidrivebase.Drivebase4MotorSparks;
import frc.robot.subsystems.multidrivebase.Drivebase4MotorTalonFX;
import frc.robot.subsystems.Collector;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    private Drivebase drivebase = Drivebase4MotorTalonFX.GetDrivebase();
    private Oi oi = new Oi(drivebase);

    
    private RobotContainer robotContainer;

    //private Arm arm;

    
    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
      //  arm = Arm.getInstance();
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
       // arm.Motor.setNeutralMode(NeutralMode.Brake);
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

    }


    @Override
    public void teleopInit() {

        //arm = Arm.getInstance();

       // arm.Motor.set(TalonFXControlMode.PercentOutput, 0);
        //arm.Motor.setNeutralMode(NeutralMode.Coast);


//        double rotateTo = 15;

//
        Command ArcadeDrive = new ArcadeDrive(drivebase, oi);
//
        ArcadeDrive.schedule();
//        arm.Motor.set(TalonFXControlMode.PercentOutput, 0);
//        arm.Motor.setNeutralMode(NeutralMode.Coast);
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
    public void testPeriodic() {}
    
    
    /** This method is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}
    
    
    /** This method is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
