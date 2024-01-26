package frc.robot.subsystems.swerve.real;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.swerve.GyroIO;


public class NavX2Gyro implements GyroIO{

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final GyroData data = new GyroData();

    public NavX2Gyro() {
        new Thread(() -> {
            try {
                while (gyro.isCalibrating()){
                    
                }
                gyro.reset();

            } catch (Exception e) {
            }
        }).start();
    }
    

    @Override
    public void updateData(GyroData data){
        data.yawDeg = gyro.getYaw();
        data.pitchDeg = gyro.getPitch();
        data.rollDeg = gyro.getRoll();
        data.isCalibrating = gyro.isCalibrating();
        data.connected = gyro.isConnected();


    }
    @Override
    public void resetGyro(){
        gyro.reset();

    }
    
}
