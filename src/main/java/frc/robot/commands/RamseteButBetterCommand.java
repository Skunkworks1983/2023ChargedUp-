package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;


public class RamseteButBetterCommand extends CommandBase {

    Pose2d start;
    Pose2d[] positions;
    Pose2d[] positionsTwo;
    Pose2d[] positionsThree;
    double speedMult;
    int soFar=0;
    boolean done=false;
    double defaultSpeed;

    public RamseteButBetterCommand(Pose2d[] positions,double defaultSpeed,double speedMult) {
        this.start=start;
        this.positions=positions;
        this.defaultSpeed=defaultSpeed;
        this.speedMult=speedMult;


        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
        /*
        for (var i=0; i<positions.length*2;i++){
            double x=(positions[(i-1)/2].getX()+positions[(i+1)/2].getX())/2;
            x+=Math.cos(positions[(i-1)/2].getRotation().getRadians())*speedMult;
            x-=Math.cos(positions[(i+1)/2].getRotation().getRadians())*speedMult;
            double y=(positions[(i-1)/2].getY()+positions[(i+1)/2].getY())/2;
            y+=Math.sin(positions[(i-1)/2].getRotation().getRadians())*speedMult;
            y-=Math.sin(positions[(i+1)/2].getRotation().getRadians())*speedMult;
            positionsTwo[i]=new Pose2d(
                    (positions[(i-1)/2].getX()+positions[(i+1)/2].getX())/2,
                    (positions[(i-1)/2].getY()+positions[(i+1)/2].getY())/2,
                    new Rotation2d(positions[(i-1)/2].getRotation().getRadians()+positions[(i+1)/2].getRotation().getRadians()/2));
            if(i%2==0)positionsTwo[i]=positions[i/2];

        }*/

    }

    @Override
    public void execute() {

        double x;//x and y are both reletive to position of the robot, but not the rotation.
        double y;//Where it should go.

    while(true) {
        x = DriveBase.getInstance().position.getX() - positions[soFar].getX();
        y = DriveBase.getInstance().position.getX() - positions[soFar].getY();
        if (Math.sqrt((x * x) + (y * y)) < 1){ soFar += 1;}else{break;}//if we get to the point, we go to the next one. If not, we break
        if(soFar>=positions.length){done=true;return;}//if we have fineshed, no need to run the next.
    }

        double r = Math.atan(y/x);
    System.out.println(r);
        r-=DriveBase.getInstance().position.getRotation().getX();
        if(r<Math.PI/16&&r>-Math.PI/16) {
            DriveBase.getInstance().setRight(defaultSpeed);
            DriveBase.getInstance().setLeft(defaultSpeed);
        }
        else if(r>0){
            DriveBase.getInstance().setRight(defaultSpeed);
            DriveBase.getInstance().setLeft(-defaultSpeed);


        }else{

            DriveBase.getInstance().setRight(-defaultSpeed);
            DriveBase.getInstance().setLeft(defaultSpeed);

        }
        //
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        DriveBase.getInstance().setRight(0);
        DriveBase.getInstance().setLeft(0);
    }
}
