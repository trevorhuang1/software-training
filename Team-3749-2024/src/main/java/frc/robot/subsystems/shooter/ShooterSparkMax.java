package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class ShooterSparkMax extends SubsystemBase {

    private CANSparkMax shooterMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax wristMotor = new CANSparkMax(1,MotorType.kBrushless);
    private PIDController wristController = new PIDController(1, 0, 0);
    private RelativeEncoder wristEncoder = wristMotor.getEncoder();
    private SimpleMotorFeedforward wristFF = new SimpleMotorFeedforward(0, 0);
    private double shooterVoltage = 0;
    private double wristVoltage = 0;

    public ShooterSparkMax() {
        System.out.println("[Init] Creating ExampleIOSim");
    }
    
   public void setShooterVolts(double volts)
   {
    this.shooterVoltage = volts;
   }

   public void stop()
   {
    shooterMotor.stopMotor();
   }

   public void setWristSetpoint(double setpoint)
   {
    wristVoltage = setpoint;
   }

   @Override
   public void periodic()
   {
    shooterMotor.setVoltage(shooterVoltage);
    wristMotor.setVoltage(wristFF.calculate(wristVoltage) + wristController.calculate(wristEncoder.getPosition(),wristVoltage));
    SmartDashboard.putNumber("shooterVolts",shooterMotor.getBusVoltage());
    SmartDashboard.putNumber("shooterWristVolts", wristMotor.getBusVoltage());
   }

}
