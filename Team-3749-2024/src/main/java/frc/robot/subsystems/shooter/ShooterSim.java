package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSim implements ShooterIO {

    private FlywheelSim shooterMotor = new FlywheelSim(DCMotor.getNEO(1),1, 0.01);
    private PIDController pidController = new PIDController(1, 0, 0);

    public ShooterSim() {
        System.out.println("[Init] Creating ExampleIOSim");
    }
    
    public void setDesiredVoltage(double volts)
    {
        shooterMotor.setInputVoltage(pidController.calculate(shooterMotor.getCurrentDrawAmps(),volts));
    }

    public void stop()
    {
        shooterMotor.setInputVoltage(pidController.calculate(shooterMotor.getCurrentDrawAmps(),0));
    }

}
