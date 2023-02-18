package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
    private Collector(){

    }
    public void intake(){

    }
    public void expel(){

    }
    public static Collector getInstance(){
        if ( instance == null){
            instance = new Collector();
        }
        return instance;
    }
    private static Collector instance;
}
