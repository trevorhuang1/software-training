package frc.robot.commands.superstructure;

import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.utils.SuperStructureStates;

public class Climb implements SuperStructureCommandInterface {

    public Climb() {

    }

    @Override
    public void start() {
        
        Robot.arm.setGoal(ArmStates.CLIMB);
        Robot.arm.moveToGoal();

    }

    @Override
    public void execute() {
        Robot.arm.setGoal(ArmStates.CLIMB);
        Robot.arm.moveToGoal();
    }
}
