package frc.robot.commands.superstructure;

import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.utils.SuperStructureStates;

public class Reset implements SuperStructureCommandInterface {

    public Reset() {

    }

    @Override
    public void start() {
        Robot.intake.setHasPiece(false);
        Robot.intake.setIndexedPiece(false);
        Robot.intake.setState(IntakeStates.STOP);
        Robot.shooter.setState(ShooterStates.STOP);
    }

    @Override
    public void execute() {
        Robot.state = SuperStructureStates.STOW;

    }
}
