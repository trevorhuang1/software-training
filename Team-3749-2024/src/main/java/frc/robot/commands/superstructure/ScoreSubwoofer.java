package frc.robot.commands.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.UtilityFunctions;

public class ScoreSubwoofer implements SuperStructureCommandInterface {
    private boolean fullDeployedWrist = false;
    private boolean almostDeployedWrist = false;
    private boolean subwoofedArm = false;
    private boolean stowedArm = false;
    private boolean staticWrist = false;


    
    public ScoreSubwoofer() {
    }

    @Override
    public void execute() {
        Robot.shooter.setState(ShooterStates.SPOOL);
        if (Robot.wrist.getState() == WristStates.FULL_DEPLOYED) {
            fullDeployedWrist = true;
        }

        if ((Robot.wrist.getState() == WristStates.ALMOST_DEPLOYED) ||
                ((Math.abs(Robot.wrist.getVelocityRadPerSec()) < 0.2) && Robot.wrist.getPositionRad() > 130)) {
            almostDeployedWrist = true;
        }

        if (Robot.arm.getState() == ArmStates.SUBWOOFER) {
            subwoofedArm = true;
        }
        if (Robot.arm.getState() == ArmStates.STOW) {
            stowedArm = true;
        }
        if (!stowedArm) {
            Robot.arm.setGoal(ArmStates.STOW);
        }

        if (!almostDeployedWrist) {
            // System.out.println("wrist to ground");
            Robot.wrist.setGoal(WristStates.ALMOST_DEPLOYED);
        }

        if ((!fullDeployedWrist && almostDeployedWrist && stowedArm)) {
            // System.out.println("arm to amp");
            Robot.arm.setGoal(ArmStates.SUBWOOFER);
            Robot.wrist.setGoal(WristStates.FULL_DEPLOYED);
            

        }
        // if (UtilityFunctions.isStopped(Robot.arm.getVelocityRadPerSec())
        //         && UtilityFunctions.isStopped(Robot.wrist.getVelocityRadPerSec()) && staticWrist) {
        //     staticWrist = false;
        // }


        // if (staticWrist) {
        //     Robot.wrist.setVoltage(1.25);
        // } else {
            Robot.wrist.moveWristToGoal();
        // }

        Robot.arm.moveToGoal();


        // SmartDashboard.putBoolean("full dep", fullDeployedWrist);

    }

    @Override
    public void reset() {
        Robot.intake.setState(IntakeStates.STOP);
        Robot.shooter.setState(ShooterStates.STOP);
        Robot.wrist.setVoltage(0);

        fullDeployedWrist = false;
        staticWrist = false;
        almostDeployedWrist = false;
        subwoofedArm = false;
        stowedArm = false;
    }

   
}
