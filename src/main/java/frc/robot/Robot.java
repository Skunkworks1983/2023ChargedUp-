// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.WaveCollectorCommandGroup;
import frc.robot.commands.autos.SmartDriveCommand;
import frc.robot.commands.autos.TestAutoTwoCommandGroup;
import frc.robot.commands.autos.TestVolocityModeCommand;
import frc.robot.commands.drivebase.TankDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.services.Oi;
import frc.robot.subsystems.Drivebase;

import java.util.List;


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


    private Command autonomousCommand;

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

    public void autonomousInit() {
        Drivebase.GetDrivebase().SetBrakeMode(true);
        //new TestAutoTwoCommandGroup().schedule();

        Drivebase.GetDrivebase().setPose(new Pose2d(Units.feetToMeters(6.33),Units.feetToMeters(23),  new Rotation2d(Math.PI)));


        new SmartDriveCommand(Constants.Autos.FirstAuto.trajectoryOne).schedule();
/*
        double distance=3;
        Trajectory exampleTrajectoryInMeters =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(distance, 0),new Translation2d(distance, -distance)
                        ,new Translation2d(0, -distance)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(0, 0, new Rotation2d(Math.PI/2)),
                        // Pass config
                        Drivebase.GetDrivebase().config);

        new SmartDriveCommand(exampleTrajectoryInMeters).schedule();

        Trajectory tOne=TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(.5, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass config
                Drivebase.GetDrivebase().config);
        for (int i=0; i<4; i++){
            Trajectory a = TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, -i, new Rotation2d(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(3, -i),new Translation2d(3, -i-1)
                            ,new Translation2d(0, -distance)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(0, -i-1, new Rotation2d(Math.PI/2)),
                    // Pass config
                    Drivebase.GetDrivebase().config);
            tOne=tOne.concatenate(a);}


        /*Trajectory exampleTrajectoryInMetersTwo =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(4, 0, new Rotation2d(Math.PI)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(2, 0.05)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass config
                        Drivebase.GetDrivebase().config);

*/


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
        Drivebase.GetDrivebase().SetBrakeMode(false);
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
