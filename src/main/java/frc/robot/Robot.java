// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveBase;

import java.util.List;
import java.util.function.BiConsumer;
//import edu.wpi.first.wpilibj;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    private Command autonomousCommand;
    
    private RobotContainer robotContainer;

    //private DriveBase driveBase;
    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
    }
    
    /**
     * This method is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic()
    {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }
    
    
    /** This method is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {


        DriveBase.getInstance().setBrakeMode(false);
    }
    
    
    @Override
    public void disabledPeriodic() {}
    
    
    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit()
    {

        DriveBase.getInstance().setBrakeMode(true);
        DriveBase.getInstance().resetOdometry();

        DriveBase.getInstance().startingPitch=DriveBase.getInstance().getPitch();
        /*
        autonomousCommand = robotContainer.getAutonomousCommand();
        
        // schedule the autonomous command (example)
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }*/
        //TurnDistancePidCommand CommandTwo = new TurnDistancePidCommand(360, .02085,10.5,1);
        //TurnDistancePidCommand CommandTwo = new TurnDistancePidCommand(360, .01085,2.5,1);//more wobble
        DriveOntoChargeStationCommand driveOntoChargeStationCommand = new DriveOntoChargeStationCommand((float).3);
        BalanceOnChargeStationCommand balanceOnChargeStationCommand = new BalanceOnChargeStationCommand(.023,0,0,.1);//works very well
        //int ticksToMove =DriveBase.getInstance().metersToticks(1f);
        //MoveDistanceCommandWithProportionalControlCommand moveDistance = new MoveDistanceCommandWithProportionalControlCommand(2048*24,-.028,.25);

        //SequentialCommandGroup group = new SequentialCommandGroup(driveOntoChargeStationCommand,moveDistance,balanceOnChargeStationCommand);
        //CommandScheduler.getInstance().schedule(balanceOnChargeStationCommand);
        //CommandScheduler.getInstance().schedule(group);
        /*Trajectory exampleTrajectoryInFeet =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(4.25, 0)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(4.25, 11+.25, new Rotation2d(Math.PI/2)),
                        // Pass config
                        DriveBase.getInstance().config);//*/

        Trajectory exampleTrajectoryInFeet =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(14+(18/12), 0)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(17, 10.5, new Rotation2d(Math.PI/2)),
                        // Pass config
                        DriveBase.getInstance().config);


        Trajectory exampleTrajectoryInFeetTwo =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(17, 10.5, new Rotation2d(Math.PI/2)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(14+(18/12), 0)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass config
                        DriveBase.getInstance().config.setReversed(true));

//exampleTrajectory

        exampleTrajectoryInFeet=exampleTrajectoryInFeet.concatenate(exampleTrajectoryInFeetTwo);
        TrajectoryConfig t = new TrajectoryConfig(Constants.getInstance().trajectoryMaxVelocity,Constants.getInstance().trajectoryMaxAcceleration);


        SmartDriveCommand drive = new SmartDriveCommand(exampleTrajectoryInFeet);


        //SequentialCommandGroup group = new SequentialCommandGroup(drive,balanceOnChargeStationCommand);

        CommandScheduler.getInstance().schedule(drive);
    }
    
    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit()
    {

        System.out.println("Hello world");
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
        RunTankDriveCommand OiCommand = new RunTankDriveCommand();
        CommandScheduler.getInstance().schedule(OiCommand);
    }
    
    
    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {


    }
    
    
    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
