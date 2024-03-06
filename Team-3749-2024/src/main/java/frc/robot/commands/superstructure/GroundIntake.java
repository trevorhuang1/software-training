package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import frc.robot.utils.SuperStructureStates;

public class GroundIntake implements SuperStructureCommandInterface {

    private boolean stowedWrist = false;
    private boolean stowedArm = false;

    public GroundIntake() {
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
    public void reset(){
        stowedArm = false;
        stowedWrist = false;
        Robot.intake.stop();
    }

 

}
