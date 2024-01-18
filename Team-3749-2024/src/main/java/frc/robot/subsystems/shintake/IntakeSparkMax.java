package frc.robot.subsystems.shintake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class IntakeSparkMax extends SubsystemBase {

    private CANSparkMax intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
    private double intakeVoltage = Constants.ShintakeConstants.idleVoltage;

    public IntakeSparkMax() {

    }
    
   public void setIntakeVolts(double volts)
   {
    this.intakeVoltage = volts;
   }

   public void stop()
   {
    intakeMotor.stopMotor();
   }

   @Override
   public void periodic()
   {
    intakeMotor.setVoltage(intakeVoltage);
    SmartDashboard.putNumber("intakeVolts",intakeMotor.getBusVoltage());
   }

}
