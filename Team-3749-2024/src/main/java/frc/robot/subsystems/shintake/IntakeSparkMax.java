package frc.robot.subsystems.shintake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class IntakeSparkMax extends SubsystemBase {

    private CANSparkMax intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax wristMotor = new CANSparkMax(1,MotorType.kBrushless);
    private PIDController wristController = new PIDController(1, 0, 0);
    private RelativeEncoder wristEncoder = wristMotor.getEncoder();
    private SimpleMotorFeedforward wristFF = new SimpleMotorFeedforward(1, 0);
    private double intakeVoltage = 0;
    private double wristVoltage = 0;
    private boolean isGroundSetpoint = true; // lets make a better way to do this
    private double wristOffset = 90;
    public IntakeSparkMax() {
        System.out.println("[Init] Creating ExampleIOSim");
    }
    
   public void setIntakeVolts(double volts)
   {
    this.intakeVoltage = volts;
   }

   public void stop()
   {
    intakeMotor.stopMotor();
   }

   public void toggleWristSetpoint()
   {
    isGroundSetpoint = !isGroundSetpoint;

    if(isGroundSetpoint)
    {
        wristVoltage = Constants.IntakeConstants.groundSetpoint;
        return;
    }
    wristVoltage = Constants.IntakeConstants.stowSetpoint;
    
   }

   @Override
   public void periodic()
   {
    intakeMotor.setVoltage(intakeVoltage);
    wristMotor.setVoltage(wristFF.calculate(wristVoltage) + wristController.calculate(wristEncoder.getPosition()+wristOffset,wristVoltage));
    SmartDashboard.putNumber("intakeVolts",intakeMotor.getBusVoltage());
    SmartDashboard.putNumber("intakeWristVotor", wristMotor.getBusVoltage());
   }

}
