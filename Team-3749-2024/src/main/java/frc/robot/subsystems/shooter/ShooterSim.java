package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSim extends SubsystemBase {

    private FlywheelSim shooterMotor = new FlywheelSim(DCMotor.getNEO(4),1, 0.01);
    private PIDController pidController = new PIDController(1, 0, 0);

    public ShooterSim() {
        System.out.println("[Init] Creating ExampleIOSim");
    }
    
    public void setDesiredVoltage(double volts)
    {
        shooterMotor.setInputVoltage(volts);//pidController.calculate(shooterMotor.getCurrentDrawAmps(),volts));
        //SmartDashboard.putNumber("shooterVolts",shooterMotor.getCurrentDrawAmps());
    }

    public void stop()
    {
        shooterMotor.setInputVoltage(0);
        //SmartDashboard.putNumber("shooterVolts",shooterMotor.getCurrentDrawAmps());
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("shooterVolts", shooterMotor.getCurrentDrawAmps());
    }

}
