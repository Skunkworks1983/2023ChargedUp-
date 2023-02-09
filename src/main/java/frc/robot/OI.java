package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.DriveBase;

public class OI {
    Joystick leftJoystick;
    Joystick rightJoystick;

    private final static OI INSTANCE = new OI();

    private OI(){
        leftJoystick = new Joystick(0);
        rightJoystick = new Joystick(1);
    }

    public static OI getInstance(){

        return INSTANCE;

    }

    public double getLeftJoystick(){

        return leftJoystick.getY();
    }

    public double getRightJoystick(){

        return rightJoystick.getY();
    }

}
