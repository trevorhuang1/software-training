package frc.robot.subsystems.shintake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.shintake.ShintakeIO.ShintakeData;
import frc.robot.utils.Constants;

public class Shintake extends SubsystemBase {

    private ShintakeIO shintakeModule;
    private ShintakeData data = new ShintakeData();
    private PIDController intakeController = new PIDController(Constants.ShintakeConstants.intakePID.kP,Constants.ShintakeConstants.intakePID.kI,Constants.ShintakeConstants.intakePID.kD);
    private PIDController shooterController = new PIDController(Constants.ShintakeConstants.shooterPID.kP, Constants.ShintakeConstants.shooterPID.kI, Constants.ShintakeConstants.shooterPID.kD);
        
    private SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(1, 0);
    private SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(1, 0);  
    
    private double intakeVelocity = 0;
    private double shooterVelocity = 0;

    public Shintake() 
        {
        shintakeModule = new ShintakeSparkMax();
         if(Robot.isSimulation()) 
         {
            shintakeModule = new ShintakeSim();
         }
    }

    public void setIntakeVelocity(double velocity)
   {
    this.intakeVelocity = velocity;
   }

   public void setShooterVelocity(double velocity)
   {
    this.shooterVelocity = velocity;
   }

   public void moveShintake()
   {
    shintakeModule.setVoltage(
        intakeController.calculate(shintakeModule.getIntakeEncoder(),intakeVelocity) + intakeFF.calculate(intakeVelocity),
        shooterController.calculate(shintakeModule.getShooterEncoder()[0],shooterVelocity) + shooterFF.calculate(shooterVelocity),
        shooterController.calculate(shintakeModule.getShooterEncoder()[1],shooterVelocity) + shooterFF.calculate(shooterVelocity)
    );
   }

    @Override
    public void periodic() {
        shintakeModule.updateData(data);
    }

}
