package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.SuperStructureStates;

public class GroundIntake extends Command {

    private boolean stowedWrist;
    private boolean stowedArm;

    public GroundIntake() {
        addRequirements(Robot.wrist, Robot.arm, Robot.intake);
    }

    @Override
    public void initialize() {
        Robot.state = SuperStructureStates.GROUND_INTAKE;

    }

    @Override
    public void execute() {
        if (Robot.wrist.getState() == WristStates.STOW) {
            stowedWrist = true;
        }
        if (!stowedWrist) {
            Robot.wrist.setGoal(WristStates.STOW);
        }
        if (Robot.arm.getState() == ArmStates.STOW) {
            stowedArm = true;
        }
        if (stowedWrist && !stowedArm) {
            Robot.arm.setGoal(ArmStates.STOW);

        }
        if (stowedWrist && stowedArm) {
            Robot.wrist.setGoal(WristStates.GROUND_INTAKE);
            Robot.intake.setIntakeVelocity(IntakeConstants.intakeVelocityRadPerSec);

        }

        Robot.arm.moveToGoal();
        Robot.wrist.moveWristToGoal();
       
    }

    @Override
    public void end(boolean interupted) {
        Robot.intake.stop();
        Robot.wrist.setGoal(WristStates.STOW);
        Robot.state = SuperStructureStates.STOW;
    }

    @Override
    public boolean isFinished() {
        return false;

    }

}
