// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.multidrivebase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivebase;


public abstract class AbstractDrivebase extends SubsystemBase
{
    /** Creates a new ExampleSubsystem. */
 public static Drivebase OGDrivebase;



    public AbstractDrivebase() {}

    public abstract void SetSpeed(double leftSpeed, double rightSpeed);

    public abstract double GetLeftDistance ();

    public abstract double GetRightDistance ();

    public abstract void SetBrakeMode (boolean enable);
    
    
    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
    
    
    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }
}
