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

        public double leftShooterVolts = 0.0;
        public double leftShooterVelocityRadPerSec = 0.0;
        public double leftShooterTempCelcius = 0.0;

        public double rightShooterVolts = 0.0;
        public double rightShooterVelocityRadPerSec = 0.0;
        public double rightShooterTempCelcius = 0.0;
        
    }
    /** Updates the set of loggable inputs. */
    public default void updateData(ShooterData data) {

    }

    /** Run the drive motor at the specified voltage. */
    public default void setVoltage(double leftShooterVolts, double rightShooterVolts) 
    {
        
    }
    
    /** Enable or disable brake mode on the drive motor. */
    public default void setBrakeMode(boolean enable) {
    }

}