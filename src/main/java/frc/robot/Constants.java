// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import frc.robot.subsystems.DriveBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    private static Constants INSTANCE;// = new Constants();

    //set these next 4 variables for smart driving
    public double kMaxSpeedMetersPerSecond=0.5;//was 3.47472
    public double kMaxAccelerationMetersPerSecondSquared=1.0792;//was 24.0792

    public double trajectoryMaxVelocity=1;

    public double trajectoryMaxAcceleration=1;

    public double kTrackwidthMeters=0.381;
    public DifferentialDriveKinematics kDriveKinematics=
            new DifferentialDriveKinematics(kTrackwidthMeters);
    public TrajectoryConstraint autoVoltageConstraint= new MaxVelocityConstraint(kMaxSpeedMetersPerSecond);


    public static Constants getInstance() {
        if(INSTANCE==null)INSTANCE = new Constants();
        return INSTANCE;
    }

    public int leftFrontMotorController=3;

    public int rightFrontMotorController=1;
    public int leftBackMotorController=4;

    public int rightBackMotorController=2;

    Pose2d BlueChargeStation;
    Pose2d RedChargeStation;
/*
    Pose3d GetGridLocation(int x, int y){//releteve to x==0 corner the most twards the center of the field.
        var p = new Pose3d;
        if(y==0) return;
        double xA=0;
        double yA=0;
        double zA=0;

        if(x%3==1)//block
            {

            }
        else{
if(y==1){yA=87;zA=58;}
if(y==2){yA=117;zA=101;}
xA=x*;

}//cone
    }*/
    /*
    Pose3d[][] redGrid = {
        {,,},
        {,,},
    {,,},
    {,,},
    {,,},
    {,,},
    {,,},
    {,,},
    ,,},

    };
    Pose3d[][] blueGrid = {
            {,,},
            {,,},
            {,,},
            {,,},
            {,,},
            {,,},
            {,,},
            {,,},
            ,,},

            };

*/
}
