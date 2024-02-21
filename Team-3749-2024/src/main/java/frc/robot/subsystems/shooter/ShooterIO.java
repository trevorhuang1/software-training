package frc.robot.subsystems.shooter;

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.


/***
 * @author Noah Simon
 *         An example for how to set up an IO class. This is to handle your
 *         motors and their encoders for a subsytems and sets up for easy
 *         logging, easy simulation, and future advantage kit implementation
 */
public interface ShooterIO {
    public static class ShooterData {

        public double bottomShooterVolts = 0.0;
        public double bottomShooterVelocityRadPerSec = 0.0;
        public double bottomShooterTempCelcius = 0.0;
        public double bottomShooterPositionRad = 0.0;
        public double bottomShooterCurrentAmps = 0.0;

        public double topShooterVolts = 0.0;
        public double topShooterVelocityRadPerSec = 0.0;
        public double topShooterTempCelcius = 0.0;
        public double topShooterPositionRad = 0.0;
        public double topShooterCurrentAmps = 0.0;
        
    }
    /** Updates the set of loggable inputs. */
    public default void updateData(ShooterData data) {

    }

    /** Run the drive motor at the specified voltage. */
    public default void setVoltage(double topShooterVolts, double bottomShooterVolts) 
    {
        
    }
    
    /** Enable or disable brake mode on the drive motor. */
    public default void setBrakeMode(boolean enable) {
    }

}