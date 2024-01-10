package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterIO.ShooterData;

public class Shooter extends SubsystemBase {

    private ShooterData data = new ShooterData();
    private ShooterIO shooterIO;
    // private Exampl

    // Constructor
    public Shooter(){
        if (Robot.isReal()){
            shooterIO = new ShooterSim();
        }
    }

    // runs every 0.02 sec
    @Override
    public void periodic(){

    }
    
}
