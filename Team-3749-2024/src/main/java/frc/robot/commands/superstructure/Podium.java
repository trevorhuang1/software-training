package frc.robot.commands.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.UtilityFunctions;

public class Podium implements SuperStructureCommandInterface {
    private boolean fullDeployedWrist = false;
    private boolean almostDeployedWrist = false;
    private boolean podiumedArm = false;
    private boolean stowedArm = false;
    private boolean staticWrist = false;

    public Podium() {
    }

    @Override
    public void execute() {
        Robot.shooter.setState(ShooterStates.SPOOL);
        if (Robot.wrist.getState() == WristStates.FULL_DEPLOYED) {
            fullDeployedWrist = true;
        }

        if ((Robot.wrist.getState() == WristStates.ALMOST_DEPLOYED) ||
                ((Math.abs(Robot.wrist.getVelocityRadPerSec()) < 0.2)
                        && Robot.wrist.getPositionRad() > WristConstants.almostDeployedRad - 0.225)) {
            almostDeployedWrist = true;
        }
        if (Robot.arm.getState() == ArmStates.PODIUM) {
            podiumedArm = true;
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
            Robot.arm.setGoal(ArmStates.PODIUM);
            Robot.wrist.setGoal(WristStates.FULL_DEPLOYED);

        }
        // if (UtilityFunctions.isStopped(Robot.arm.getVelocityRadPerSec())
        // && UtilityFunctions.isStopped(Robot.wrist.getVelocityRadPerSec()) &&
        // staticWrist) {
        // staticWrist = false;
        // }

        // if (staticWrist) {
        // Robot.wrist.setVoltage(1.25);
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
        podiumedArm = false;
        stowedArm = false;
        atIntake = false;
    }

    private boolean atIntake = false;

    @Override
    public void autoExecute() {
        Robot.shooter.setState(ShooterStates.SPOOL);
        if (Robot.wrist.getState() == WristStates.FULL_DEPLOYED) {
            fullDeployedWrist = true;
        }

        if ((Robot.wrist.getState() == WristStates.ALMOST_DEPLOYED) ||
                ((Math.abs(Robot.wrist.getVelocityRadPerSec()) < 0.2)
                        && Robot.wrist.getPositionRad() > WristConstants.almostDeployedRad - 0.225)) {
            almostDeployedWrist = true;
        }
        if (Robot.arm.getState() == ArmStates.PODIUM) {
            podiumedArm = true;
        }
        if (Robot.arm.getState() == ArmStates.STOW) {
            stowedArm = true;
        }
        if (!stowedArm && !fullDeployedWrist) {
            Robot.arm.setGoal(ArmStates.STOW);
        }

        if (!almostDeployedWrist && !fullDeployedWrist) {
            // System.out.println("wrist to ground");
            Robot.wrist.setGoal(WristStates.ALMOST_DEPLOYED);
        }

        if ((!fullDeployedWrist && almostDeployedWrist && stowedArm)) {
            // System.out.println("arm to amp");
            Robot.arm.setGoal(ArmStates.PODIUM);
            Robot.wrist.setGoal(WristStates.FULL_DEPLOYED);

        }
        if (fullDeployedWrist){
            Robot.arm.setGoal(ArmStates.PODIUM);
            Robot.wrist.setGoal(WristStates.FULL_DEPLOYED);
        }
        // if (UtilityFunctions.isStopped(Robot.arm.getVelocityRadPerSec())
        // && UtilityFunctions.isStopped(Robot.wrist.getVelocityRadPerSec()) &&
        // staticWrist) {
        // staticWrist = false;
        // }

        // if (staticWrist) {
        // Robot.wrist.setVoltage(1.25);
        // } else {
        Robot.wrist.moveWristToGoal();
        // }

        Robot.arm.moveToGoal();

        // SmartDashboard.putBoolean("full dep", fullDeployedWrist);
    }
    @Override
    public void autoReset() {
        reset();
        atIntake = false;
    }
    @Override
    public void autoStart(){
        start();
    }
}
