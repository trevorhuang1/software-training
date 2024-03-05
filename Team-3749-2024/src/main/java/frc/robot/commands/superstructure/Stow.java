package frc.robot.commands.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
    private boolean armWasRaised = false;
    private boolean stowedWrist = false;

    private Timer timer = new Timer();

    public Stow() {
    }

    private double maxArmRadForStow = Units.degreesToRadians(15);

    @Override
    public void execute() {

        if (Robot.arm.getPositionRad() > maxArmRadForStow) {
            Robot.arm.setGoal(Units.degreesToRadians(10));
            armWasRaised = true;
        } else {
            loweredArm = true;

        }

        if (Robot.wrist.getState() == WristStates.STOW) {
            stowedWrist = true;
        }
        if (!stowedWrist && loweredArm) {
            if (armWasRaised) {

                if (timer.get() == 0) {
                    timer.start();
                }
                if (timer.get() > 1) {

                    Robot.wrist.setGoal(WristStates.STOW);
                }
            } else {
                Robot.wrist.setGoal(WristStates.STOW);

            }
        }
        if (stowedWrist) {
            Robot.arm.setGoal(ArmStates.STOW);

        }

        Robot.arm.moveToGoal();
        Robot.wrist.moveWristToGoal();
        Robot.intake.stop();
        Robot.shooter.stop();

        SmartDashboard.putBoolean("lowering arm", loweringArm);
        SmartDashboard.putBoolean("lowered arm", loweredArm);
        SmartDashboard.putBoolean("Stowed wrist", stowedWrist);

    }

    @Override
    public void reset() {
        stowedWrist = false;
        loweredArm = false;
        loweringArm = false;
        armWasRaised = false;
        timer.stop();
        timer.reset();
    }

}
