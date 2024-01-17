package frc.robot.subsystems.shintake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSim extends SubsystemBase {

    private FlywheelSim intakeMotor = new FlywheelSim(DCMotor.getNEO(1),1, 0.01);
    private FlywheelSim wristMotor = new FlywheelSim(DCMotor.getNEO(1), 1, 0.01);
    private PIDController wristController = new PIDController(1, 0, 0);
    private Mechanism2d wristMech = new Mechanism2d(3, 3);
    private MechanismRoot2d wristRoot = wristMech.getRoot("wrist", 0, 1);
    private MechanismLigament2d wrist = wristRoot.append(new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
    private MechanismLigament2d intakeT = wrist.append(new MechanismLigament2d("wrist", 0.5, -90, 6, new Color8Bit(Color.kGreen)));
    private double wristAngle = 0;

    public IntakeSim() {  
        System.out.println("[Init] Creating ExampleIOSim");
    }
    
   public void setIntakeVolts(double volts)
   {
    intakeMotor.setInputVoltage(volts);
   }

   public void setWristSetpoint()
   {
    //wristController.calculate(, 0);
   }

   public void moveWristAngle(double increase)
   {
    wristAngle += increase * 2;
    wristAngle = MathUtil.clamp(wristAngle, 0, 90);
    //System.out.println(wristAngle);
   }

   @Override
   public void periodic()
   {
    wrist.setAngle(wristAngle);
    SmartDashboard.putNumber("intakeVolts",intakeMotor.getCurrentDrawAmps());
    SmartDashboard.putData("Mech2d", wristMech);
   }

}
