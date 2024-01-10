package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSim extends SubsystemBase {

    private FlywheelSim intakeMotor = new FlywheelSim(DCMotor.getNEO(1),1, 0.01);

    public IntakeSim() {
        System.out.println("[Init] Creating ExampleIOSim");
    }
    
   public void setIntakeVolts(double volts)
   {
    intakeMotor.setInputVoltage(volts);
   }

   @Override
   public void periodic()
   {
    SmartDashboard.putNumber("intakeVolts",intakeMotor.getCurrentDrawAmps());
   }

}
