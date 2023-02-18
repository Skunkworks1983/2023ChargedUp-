package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Objects;
import java.util.Set;

public class Limelight extends SubsystemBase {

    private final static Limelight INSTANCE = new Limelight();
    public static Limelight GetInstance(){
        return INSTANCE;
    }
    public double x,y;
    double ta;
    private double area;
    private double skew;
public static double poseData[];
public static int currentTag;
    private double xOffset, yOffset;
    private double pitch, yaw, roll;
    public static final boolean TARGET_DETECTED_DEFAULT_VALUE = false;
    public static final double X_DEFAULT_VALUE = 0.0;
    public static final double Y_DEFAULT_VALUE = 0.0;
    public static final double AREA_DEFAULT_VALUE = 0.0;
    public static final double SKEW_DEFAULT_VALUE = 0.0;
    public static final double X_OFFSET_DEFAULT_VALUE = 0.0;
    public static final double Y_OFFSET_DEFAULT_VALUE = 0.0;
    public static final double PITCH_DEFAULT_VALUE = 0.0;
    public static final double YAW_DEFAULT_VALUE = 0.0;
    public static final double ROLL_DEFAULT_VALUE = 0.0;
    public static final double LED_MODE_DEFAULT_VALUE = 0.0;


    public Limelight(){
        //new Thread(this).start();
    }


    public static void UpdateTable() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
/*
        x = table.getEntry("tx").getDouble(X_DEFAULT_VALUE);
        y = table.getEntry("ty").getDouble(Y_DEFAULT_VALUE);
        ta = table.getEntry("ta").getDouble(Y_DEFAULT_VALUE);
        area = table.getEntry("ta").getDouble(AREA_DEFAULT_VALUE);
        skew = table.getEntry("ts").getDouble(SKEW_DEFAULT_VALUE);*/
        poseData = table.getEntry("botpose").getDoubleArray(new double[6]);
        currentTag = (int)table.getEntry("tid").getInteger((int)-1);
        /*Object[] keys = table.getTopics().toArray();
        System.out.println(keys.length + "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW");
        for(int i=0;i<keys.length;i++){

            System.out.println((String)keys[i]);

        }*/

    }
}
