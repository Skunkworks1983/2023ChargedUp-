package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.FlipFieldHelper.flipPose;
import static frc.robot.FlipFieldHelper.flipTranslation;

public class UnconstructedTrajectory {
    List<Translation2d> goThrough;
    Pose2d finalPose;
    boolean reversed;
    public UnconstructedTrajectory(List<Translation2d> goThrough,Pose2d finalPose,boolean reversed){

        this.goThrough=goThrough;
        this.finalPose=finalPose;
        this.reversed=reversed;
    }

    public List<Translation2d> getGoThroughTranslations(){return goThrough;}

    public Pose2d getEndingPose(){return finalPose;}

    public boolean getReversed(){return reversed;}

    public UnconstructedTrajectory flipped(){
        UnconstructedTrajectory traj;
        List<Translation2d> goThrough=new ArrayList<>();
        Pose2d finalPose;
        finalPose= flipPose(this.finalPose);
        flipPose(this.finalPose);
        for(int i =0; i<this.goThrough.toArray().length;i++){
            goThrough.add(flipTranslation(this.goThrough.get(i)));
        }

        traj = new UnconstructedTrajectory(goThrough,finalPose,reversed);
        return traj;
    }



}
