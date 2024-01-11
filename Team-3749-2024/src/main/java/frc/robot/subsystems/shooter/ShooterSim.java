package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSim extends SubsystemBase {

    private FlywheelSim shooterMotor = new FlywheelSim(DCMotor.getNEO(1),1, 0.01);
    // private PIDController pidController = new PIDController(1, 0, 0);
    private double voltageGoal = 0;

    public ShooterSim() {
        System.out.println("[Init] Creating ExampleIOSim");
    }
    
    public void setDesiredVoltage(double volts)
    {
        voltageGoal = volts;
    }

    public void stop()
    {
        voltageGoal = 0;
        //shooterMotor.setInputVoltage(0);
        //SmartDashboard.putNumber("shooterVolts",shooterMotor.getCurrentDrawAmps());
    }

    @Override
    public void periodic()
    {
        voltageGoal = MathUtil.clamp(voltageGoal, -8, 8);
        shooterMotor.setInputVoltage(voltageGoal);
        SmartDashboard.putNumber("shooterVolts", shooterMotor.getCurrentDrawAmps());
    }

}
