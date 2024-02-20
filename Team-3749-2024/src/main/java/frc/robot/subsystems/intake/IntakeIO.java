package frc.robot.subsystems.intake;

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
public interface IntakeIO {
    public static class IntakeData {
        public double intakeVolts = 0.0;
        public double intakeVelocityRadPerSec = 0.0;
        public double intakeTempCelcius = 0.0;
        public double currentAmps = 0.0;
        public boolean sensorTripped = false;
    }
    /** Updates the set of loggable inputs. */
    public default void updateData(IntakeData data) {

    }

    /** Run the drive motor at the specified voltage. */
    public default void setVoltage(double intakeVolts) 
    {
        
    }

    /** Enable or disable brake mode on the drive motor. */
    public default void setBrakeMode(boolean enable) {
    }

}