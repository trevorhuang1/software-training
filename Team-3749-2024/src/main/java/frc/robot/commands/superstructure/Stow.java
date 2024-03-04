package frc.robot.commands.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.SuperStructureStates;
import frc.robot.utils.UtilityFunctions;

public class Stow implements SuperStructureCommandInterface {
    private boolean loweringArm = false;
    private boolean loweredArm = false;
    private boolean stowedWrist = false;

    public Stow() {
    }

    @Override
    public void execute() {

        if (Robot.arm.getPositionRad() > Units.degreesToRadians(45)) {
            Robot.arm.setGoal(Units.degreesToRadians(20));
            loweringArm = true;
        } else if (!loweringArm){
            loweredArm = true;
        }

        if (loweringArm && UtilityFunctions.withinMargin(0.052, Robot.arm.getPositionRad(), Units.degreesToRadians(20))){
            loweredArm = true;
        }



        if (Robot.wrist.getState() == WristStates.STOW) {
            stowedWrist = true;
        }
        if (!stowedWrist && loweredArm) {
            Robot.wrist.setGoal(WristStates.STOW);
        }

        if (stowedWrist) {
            Robot.arm.setGoal(ArmStates.STOW);

        }
        Robot.arm.moveToGoal();
        Robot.wrist.moveWristToGoal();
        Robot.intake.stop();
        Robot.shooter.stop();

    }

    @Override
    public void reset() {
        stowedWrist = false;
    }

}
