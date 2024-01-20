package frc.robot.subsystems.wrist;

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
public interface WristIO {
    public static class WristData { 
        public double wristAngle = 0.0;
        public double wristVoltage = 0.0;
        public double tempCelcius = 0.0;
        public double targetAngle = 0.0;
    }
    /** Updates the set of loggable inputs. */
    public default void updateData(WristData data) {

    }

    /** Run the drive motor at the specified voltage. */
    public default void setVoltage(double volts) {
        
    }

    /** Enable or disable brake mode on the drive motor. */
    public default void setBrakeMode(boolean enable) {
    }

}