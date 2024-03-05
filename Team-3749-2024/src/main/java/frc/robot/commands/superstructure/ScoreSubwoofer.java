package frc.robot.commands.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.UtilityFunctions;

public class ScoreSubwoofer implements SuperStructureCommandInterface {
    private boolean fullDeployedWrist = false;
    private boolean groundIntakeWrist = false;
    private boolean subwoofedArm = false;
    private boolean stowedArm = false;
    private boolean staticWrist = false;


    
    public ScoreSubwoofer() {
    }

    @Override
    public void execute() {

        if (Robot.wrist.getState() == WristStates.FULL_DEPLOYED) {
            fullDeployedWrist = true;
        }

        if ((Robot.wrist.getState() == WristStates.GROUND_INTAKE) ||
                ((Math.abs(Robot.wrist.getVelocityRadPerSec()) < 0.2) && Robot.wrist.getPositionRad() > 130)) {
            groundIntakeWrist = true;
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

        if (!fullDeployedWrist && !groundIntakeWrist && stowedArm) {
            // System.out.println("wrist to ground");
            Robot.wrist.setGoal(WristStates.GROUND_INTAKE);
        }

        if ((!fullDeployedWrist && groundIntakeWrist && stowedArm)
                || (fullDeployedWrist)) {
            // System.out.println("arm to amp");
            Robot.arm.setGoal(ArmStates.SUBWOOFER);
            staticWrist = true;
            if (Robot.arm.getPositionRad() > Units.degreesToRadians(5)) {
                Robot.wrist.setGoal(WristStates.FULL_DEPLOYED);
            }

        }
        if (UtilityFunctions.isStopped(Robot.arm.getVelocityRadPerSec())
                && UtilityFunctions.isStopped(Robot.wrist.getVelocityRadPerSec()) && staticWrist) {
            staticWrist = false;
        }


        if (staticWrist) {
            Robot.wrist.setVoltage(1.25);
        } else {
            Robot.wrist.moveWristToGoal();
        }

        Robot.arm.moveToGoal();

        SmartDashboard.putBoolean("full dep", fullDeployedWrist);
        SmartDashboard.putBoolean("ground int", groundIntakeWrist);
        SmartDashboard.putBoolean("amp arm", subwoofedArm);
        SmartDashboard.putBoolean("amp stow", stowedArm);
        // SmartDashboard.putBoolean("full dep", fullDeployedWrist);

    }

    @Override
    public void reset() {
        Robot.intake.stop();
        Robot.shooter.stop();
        Robot.wrist.setVoltage(0);

        fullDeployedWrist = false;
        staticWrist = false;
        groundIntakeWrist = false;
        subwoofedArm = false;
        stowedArm = false;
    }
}
