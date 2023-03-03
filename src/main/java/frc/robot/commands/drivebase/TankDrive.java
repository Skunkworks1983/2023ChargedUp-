// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.services.Oi;
import frc.robot.subsystems.drivebase.Drivebase4MotorTalonFX;


public class TankDrive extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivebase4MotorTalonFX drivebase;
    private final Oi oi;

    public TankDrive(Drivebase4MotorTalonFX drivebase, Oi oi)
    {
        this.drivebase = drivebase;
        addRequirements(drivebase);
        this.oi = oi;
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        double outputLeft = oi.getLeftY(); //motors are running backwards and are on the wrong sides
        double outputRight = oi.getRightY();
        if(outputLeft > 0)
        {
            outputLeft = Math.pow(outputLeft, 2)/4;
        }
        else
        {
            outputLeft = -Math.abs(Math.pow(outputLeft, 2)/4);
        }
        if(outputRight > 0)
        {
            outputRight = Math.pow(outputRight, 2)/4;
        }
        else
        {
            outputRight = -Math.abs(Math.pow(outputRight, 2)/4);
        }
        drivebase.runMotor(-outputLeft, -outputRight);
        //System.out.println("left: " + -outputLeft + " right: " + -outputRight);
        //System.out.println("ticks: " + drivebase.getTicksLeft());
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
