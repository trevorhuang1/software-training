package frc.robot.commands.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.SuperStructureStates;
import frc.robot.utils.UtilityFunctions;

public class ScoreAmp implements SuperStructureCommandInterface {
    private boolean fullDeployedWrist = false;
    private boolean almostDeployedWrist = false;
    private boolean ampedArm = false;
    private boolean stowedArm = false;
    private boolean staticWrist = false;

    public ScoreAmp() {
    }

    @Override
    public void execute() {

        // if (Robot.wrist.getState() == WristStates.FULL_DEPLOYED) {
        // fullDeployedWrist = true;
        // }

        // if ((Robot.wrist.getState() == WristStates.ALMOST_DEPLOYED) ||
        // ((Math.abs(Robot.wrist.getVelocityRadPerSec()) < 0.2) &&
        // Robot.wrist.getPositionRad() > WristConstants.almostDeployedRad - 0.225)) {
        // almostDeployedWrist = true;
        // }
        // if (Robot.arm.getState() == ArmStates.AMP) {
        // ampedArm = true;
        // }
        // if (Robot.arm.getState() == ArmStates.STOW) {
        // stowedArm = true;
        // }
        // if (!stowedArm) {
        // Robot.arm.setGoal(ArmStates.STOW);
        // }

        // if (!fullDeployedWrist && !almostDeployedWrist && stowedArm) {
        // // System.out.println("wrist to ground");
        // Robot.wrist.setGoal(WristStates.ALMOST_DEPLOYED);
        // }

        // if ((!fullDeployedWrist && almostDeployedWrist && stowedArm)) {
        // // System.out.println("arm to amp");
        // Robot.arm.setGoal(ArmStates.AMP);
        // staticWrist = true;
        // if (Robot.arm.getPositionRad() > Units.degreesToRadians(5)) {
        // Robot.wrist.setGoal(WristStates.FULL_DEPLOYED);
        // }

        // }
        // if (UtilityFunctions.isStopped(Robot.arm.getVelocityRadPerSec())
        // && UtilityFunctions.isStopped(Robot.wrist.getVelocityRadPerSec()) &&
        // staticWrist) {
        // staticWrist = false;
        // }

        // if (ampedArm && fullDeployedWrist) {

        // }
        // if (staticWrist) {
        // Robot.wrist.setVoltage(1.5);
        // } else {
        // Robot.wrist.moveWristToGoal();
        // }

        // Robot.arm.moveToGoal();

        // SmartDashboard.putBoolean("full dep", fullDeployedWrist);


        Robot.wrist.setGoal(WristStates.STOW);
        Robot.arm.setGoal(ArmStates.AMP);
        
        Robot.arm.moveToGoal();
        Robot.wrist.moveWristToGoal();
    }

    @Override
    public void reset() {
        Robot.intake.stop();
        Robot.shooter.stop();
        Robot.wrist.setVoltage(0);

        fullDeployedWrist = false;
        staticWrist = false;
        almostDeployedWrist = false;
        ampedArm = false;
        stowedArm = false;
    }

}
