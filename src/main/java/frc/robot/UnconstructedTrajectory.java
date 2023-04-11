package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

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

}
