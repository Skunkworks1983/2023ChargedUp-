// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import java.util.ArrayList;


public class LimeLight extends SubsystemBase
{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-piece");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tl = table.getEntry("tl");
    NetworkTableEntry cl = table.getEntry("cl");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry botpose_wpiblue = table.getEntry("botpose_wpiblue");
    NetworkTableEntry botpose_wpired = table.getEntry("botpose_wpired");

    ArrayList<Double> listA = new ArrayList<>();
    ArrayList<Double> listX = new ArrayList<>();
    ArrayList<Double> listY = new ArrayList<>();

    private static LimeLight limeLightInstance;

    double averageX;
    double averageY;
    double averageA;

    boolean enable;

    public LimeLight()
    {

    }


    @Override
    public void periodic()
    {
        if(!enable)
        {
            return;
        }
        //x rolling average

        double limeX = tx.getDouble(0.0); //sets limeX to current x value
        listX.add(limeX);
        if(listX.size() > Constants.Drivebase.ROLLING_AVERAGE_LENGTH)
        {
            listX.remove(0);
        }

        double sumX = 0;

        for(double listItem : listX)
        {
            sumX = sumX + listItem;
        }

        averageX = sumX / listX.size();


        //y rolling average

        double limeY = ty.getDouble(0.0); //sets limeY to current y value
        listY.add(limeY);
        if(listY.size() > Constants.Drivebase.ROLLING_AVERAGE_LENGTH)
        {
            listY.remove(0);
        }

        double sumY = 0;

        for(double listItem : listY)
        {
            sumY = sumY + listItem;
        }

        averageY = sumY / listY.size();


        //a rolling average

        double limeA = ta.getDouble(0.0); //sets limeA to current a value
        listA.add(limeA);
        if(listA.size() > Constants.Drivebase.ROLLING_AVERAGE_LENGTH)
        {
            listA.remove(0);
        }

        double sumA = 0;

        for(double listItem : listA)
        {
            sumA = sumA + listItem;
        }

        averageA = sumA / listA.size();
    }

    public void setEnable(boolean enable)
    {
        this.enable = enable;
    }

    public double getLimeX()
    {

        if(enable)
        {

            return averageX;

        }
        else
        {

            return 0;
        }
    }

    public double getLimeY()
    {

        if(enable)
        {

            return averageY;
        }
        else
        {

            return 0;
        }
    }


    public double getLimeA()
    {

        if(enable)
        {

            return averageA;
        }
        else
        {

            return 0;
        }
    }


    public static LimeLight getInstance()
    {

        if(limeLightInstance == null)
        {
            limeLightInstance = new LimeLight();
        }

        return limeLightInstance;

    }
}
