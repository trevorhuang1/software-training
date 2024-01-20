package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.wrist.WristIO.WristData;
import frc.robot.utils.Constants;


/*
 * note from jonathan:
 * always start the robot with the robot stowed!!!!
 * 
 * if you do not i will be very sad (and it also might destory itself)
 */

public class WristSim implements WristIO {
    

    private FlywheelSim wristMotor = new FlywheelSim(DCMotor.getNEO(1),1, 0.01);

    private Encoder wristEncoder = new Encoder(0,1); //idk what this means tbh

    private EncoderSim wristEncoderSim = new EncoderSim(wristEncoder);
    private PIDController wristController = new PIDController(1, 0, 0);
    private SimpleMotorFeedforward wristFF = new SimpleMotorFeedforward(1, 0);
    private double targetAngle = Constants.WristConstants.stowSetpoint;

    public WristSim() 
    {
        
    }

   public void setWristAngle(double angle)
   {
    this.targetAngle = angle;
   }

   public void moveWristAngle()
   {
    wristMotor.setInputVoltage(
        wristController.calculate(wristEncoder.getPosition(),targetAngle) + 
        wristFF.calculate(targetAngle)
    );
   }

   @Override
   public void updateData(WristData data) 
   {
    data.tempCelcius = 0;
    data.wristAngle = wristEncoder.getPosition();
    data.targetAngle = this.targetAngle;
    data.wristVoltage = wristMotor.getCurrentDrawAmps();
   }

   //called by Wrist Subsystem's periodic : notice the lack of override
   public void periodic()
   {
    moveWristAngle();
   }

}
