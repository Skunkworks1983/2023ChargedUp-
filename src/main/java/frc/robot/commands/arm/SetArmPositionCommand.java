package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.services.Oi;
import frc.robot.subsystems.Arm;

public class SetArmPositionCommand extends CommandBase
{

    private final Arm arm;
    private final Oi oi;
    private double shoulderAngleSetpoint;
    private double wristAngleSetpoint;
    private Arm.ArmPosition armPosition;
    private Arm.PostionPieceType postionPieceType;
    private boolean weirdAngle;
    private boolean isCubeTrue;
    public SetArmPositionCommand(Arm.ArmPosition armPosition, Arm.PostionPieceType postionPieceType)
    {
        this.armPosition = armPosition;
        this.postionPieceType = postionPieceType;


        oi = Oi.Instance;
        this.arm = Arm.getInstance();
        isCubeTrue = oi.getCubeToggle();


        addRequirements(arm);
        weirdAngle = false;
    }

    @Override
    public void initialize()
    {
        double shoulderAngleSetpoint;
        double wristAngleSetpoint;
        arm.setCurrentPosition(armPosition);

        if(postionPieceType == Arm.PostionPieceType.CONE) {
            isCubeTrue = false;
        }
        if(postionPieceType == Arm.PostionPieceType.CUBE) {
            isCubeTrue = true;
        }
        if(postionPieceType == Arm.PostionPieceType.CHECK_OI) {
            isCubeTrue = oi.getCubeToggle();
        }
        if(postionPieceType == Arm.PostionPieceType.DOESNT_MATTER) {
            isCubeTrue = true;
        }
        if(armPosition == Arm.ArmPosition.FLOOR) {
            if(isCubeTrue) {
                shoulderAngleSetpoint = Constants.ArmPos.FLOOR_CUBE_PICKUP_SHOULDER;
                wristAngleSetpoint = Constants.ArmPos.FLOOR_CUBE_PICKUP_WRIST;
            }
            else {
                shoulderAngleSetpoint = Constants.ArmPos.CONE_FLOOR_PICKUP_SHOULDER;
                wristAngleSetpoint = Constants.ArmPos.CONE_FLOOR_PICKUP_WRIST;
            }
        }
        else if(armPosition == Arm.ArmPosition.FLOOR_WEIRD) {
            shoulderAngleSetpoint = Constants.ArmPos.SCORE_CONE_WEIRD_SHOULDER;
            wristAngleSetpoint = Constants.ArmPos.SCORE_CONE_WEIRD_WRIST;
        }
        else if(armPosition == Arm.ArmPosition.SUBSTATION) {
            if(isCubeTrue) {
                shoulderAngleSetpoint = Constants.ArmPos.PLAYER_CUBE_PICKUP_SHOULDER;
                wristAngleSetpoint = Constants.ArmPos.PLAYER_CUBE_PICKUP_WRIST;
            }
            else{
                shoulderAngleSetpoint = Constants.ArmPos.PLAYER_CONE_PICKUP_SHOULDER;
                wristAngleSetpoint = Constants.ArmPos.PLAYER_CONE_PICKUP_WRIST;
            }

        }
        else if(armPosition == Arm.ArmPosition.HIGH_CUBE) {
            shoulderAngleSetpoint = Constants.ArmPos.SCORE_CUBE_HIGH_SHOULDER;
            wristAngleSetpoint = Constants.ArmPos.SCORE_CUBE_HIGH_WRIST;
        }
        else if(armPosition == Arm.ArmPosition.FLOOR_NORMAL) {
            shoulderAngleSetpoint = Constants.ArmPos.FLOOR_NORMAL_SCORE_SHOULDER;
            wristAngleSetpoint = Constants.ArmPos.FLOOR_NORMAL_SCORE_WRIST;
        }
        else if(armPosition == Arm.ArmPosition.SCORE_MID) {
            if(isCubeTrue) {
                shoulderAngleSetpoint = Constants.ArmPos.SCORE_CUBE_MID_SHOULDER;
                wristAngleSetpoint = Constants.ArmPos.SCORE_CUBE_MID_WRIST;
            }
            else{
                shoulderAngleSetpoint = Constants.ArmPos.SCORE_CONE_MID_SHOULDER;
                wristAngleSetpoint = Constants.ArmPos.SCORE_CONE_MID_WRIST;
            }
        }
        else {
            shoulderAngleSetpoint = Constants.ArmPos.CARRY_SHOULDER;
            wristAngleSetpoint = Constants.ArmPos.CARRY_WRIST;
        }

        this.shoulderAngleSetpoint = shoulderAngleSetpoint;
        this.wristAngleSetpoint = wristAngleSetpoint;
        if(Math.abs(arm.ShoulderMotor.getClosedLoopTarget() * Constants.Arm.SHOULDER_TICKS_TO_DEGREES - Constants.ArmPos.SCORE_CONE_WEIRD_SHOULDER) < 1)
        {
            weirdAngle = true;
            arm.setWristAnglePosition(wristAngleSetpoint);
        }
        else
        {
            arm.setWristAnglePosition(wristAngleSetpoint);
            arm.setShoulderAnglePosition(shoulderAngleSetpoint);
            weirdAngle = false;
        }
            System.out.println("set arm pos with wristAngleSetpoint: " + wristAngleSetpoint + " and shoulderAngleSetpoint: " + shoulderAngleSetpoint);
    }

    @Override
    public void execute()
    {
        if(weirdAngle && arm.getWristAngle() < Constants.Arm.WRIST_PARALLEL_WITH_SHOULDER)
        {
            arm.setShoulderAnglePosition(shoulderAngleSetpoint);
            weirdAngle = false;
        }
    }

    @Override
    public boolean isFinished()
    {
        return false;
        //return Math.abs(arm.ShoulderMotor.getClosedLoopError() * Constants.Arm.SHOULDER_TICKS_TO_DEGREES) < Constants.Arm.SHOULDER_TOLERANCE &&
        //Math.abs(arm.WristMotor.getClosedLoopError() * Constants.Arm.WRIST_TICKS_TO_DEGREES) < Constants.Arm.WRIST_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted)
    {
        if(interrupted)
        {
            System.out.println("SetArmPositionCommand Ending, interrupted");
        }
        else
        {
            System.out.println("SetArmPositionCommand Ending");
        }
    }
}
