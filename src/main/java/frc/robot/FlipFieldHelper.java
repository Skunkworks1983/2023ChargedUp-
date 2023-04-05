package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;

public class FlipFieldHelper {

    public static Translation2d flipTranslation(Translation2d translation){
        Translation2d t = new Translation2d(
                translation.getX(),
                (Constants.FieldMeasurements.FieldYFeetTotal/Constants.Drivebase.FEET_PER_METER)-translation.getY()
        );
        return t;

    }

    public static Pose2d flipPose(Pose2d pose){

        Pose2d newPose = new Pose2d(
                pose.getX(),
                (Constants.FieldMeasurements.FieldYFeetTotal/Constants.Drivebase.FEET_PER_METER)-pose.getY(),
                new Rotation2d(-pose.getRotation().getRadians()));
        return newPose;
    }
}
