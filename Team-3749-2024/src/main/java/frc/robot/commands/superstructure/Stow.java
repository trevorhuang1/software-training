package frc.robot.commands.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.led.LEDConstants.LEDPattern;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.wrist.WristConstants;
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

    private double maxArmRadForStow = Units.degreesToRadians(8);

    @Override
    public void execute() {

        if (Robot.arm.getPositionRad() > maxArmRadForStow) {
            Robot.arm.setGoal(Units.degreesToRadians(6));
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
            if (Robot.intake.getState() == IntakeStates.INTAKE) {
                Robot.intake.setState(IntakeStates.STOP);
                Robot.shooter.setState(ShooterStates.STOP);

            }


        }

        // if (Robot.wrist.getState() == WristStates.IN_TRANIST && Robot.arm.getState()
        // == ArmStates.STOW
        // && Robot.wrist.getWristGoal().position == WristConstants.stowGoalRad) {
        // Robot.wrist.setGoal(WristStates.GROUND_INTAKE);
        // }

        Robot.arm.moveToGoal();
        Robot.wrist.moveWristToGoal();

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

    @Override
    public void start() {
        Robot.intake.stop();
        Robot.shooter.stop();

    }


}
