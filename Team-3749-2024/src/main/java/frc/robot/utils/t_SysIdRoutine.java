package frc.robot.utils;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModule;

public class t_SysIdRoutine {

  public static SysIdRoutine getSysIdRoutine(SwerveModule[] modules, Swerve subsystem, String subsystemName) {
    String name = (subsystemName.toLowerCase().replace(" ", "")) + "-";

    final MutableMeasure<Voltage> identificationVoltageMeasure = mutable(
        Volts.of(0));
    final MutableMeasure<Distance> identificationDistanceMeasure = mutable(
        Meters.of(0));
    final MutableMeasure<Velocity<Distance>> identificaitonVelocityMeasure = mutable(
        MetersPerSecond.of(0));

    return new SysIdRoutine(
        // new SysIdRoutine.Config(),
        new SysIdRoutine.Config(
            Volts.per(Seconds).of(1),
            Volts.of(7),
            Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            subsystem::identificationDriveConsumer,
            log -> {
              // Record a frame for the left motors. Since these share an encoder, we consider
              // the entire group to be one motor.
              SmartDashboard.putNumber(
                  name + "motorAppliedVolts",
                  identificationVoltageMeasure
                      .mut_replace(modules[0].getModuleData().driveAppliedVolts, Volts)
                      .magnitude());
              SmartDashboard.putNumber(
                  name + "motorSpeed",
                  identificaitonVelocityMeasure
                      .mut_replace(
                          modules[0].getModuleData().driveVelocityMPerSec,
                          MetersPerSecond)
                      .magnitude());

              log
                  .motor(name + "front-left")
                  .voltage(
                      identificationVoltageMeasure.mut_replace(
                          modules[0].getModuleData().driveAppliedVolts,
                          Volts))
                  .linearPosition(
                      identificationDistanceMeasure.mut_replace(
                          modules[0].getModuleData().drivePositionM,
                          Meters))
                  .linearVelocity(
                      identificaitonVelocityMeasure.mut_replace(
                          modules[0].getModuleData().driveVelocityMPerSec,
                          MetersPerSecond));
              // Record a frame for the right motors. Since these share an encoder, we
              // consider
              // the entire group to be one motor.
              log
                  .motor(name + "front-right")
                  .voltage(
                      identificationVoltageMeasure.mut_replace(
                          modules[1].getModuleData().driveAppliedVolts,
                          Volts))
                  .linearPosition(
                      identificationDistanceMeasure.mut_replace(
                          modules[1].getModuleData().drivePositionM,
                          Meters))
                  .linearVelocity(
                      identificaitonVelocityMeasure.mut_replace(
                          modules[1].getModuleData().driveVelocityMPerSec,
                          MetersPerSecond));

              log
                  .motor(name + "back-left")
                  .voltage(
                      identificationVoltageMeasure.mut_replace(
                          modules[2].getModuleData().driveAppliedVolts,
                          Volts))
                  .linearPosition(
                      identificationDistanceMeasure.mut_replace(
                          modules[2].getModuleData().drivePositionM,
                          Meters))
                  .linearVelocity(
                      identificaitonVelocityMeasure.mut_replace(
                          modules[2].getModuleData().driveVelocityMPerSec,
                          MetersPerSecond));
              log
                  .motor(name + "back-right")
                  .voltage(
                      identificationVoltageMeasure.mut_replace(
                          modules[3].getModuleData().driveAppliedVolts,
                          Volts))
                  .linearPosition(
                      identificationDistanceMeasure.mut_replace(
                          modules[3].getModuleData().drivePositionM,
                          Meters))
                  .linearVelocity(
                      identificaitonVelocityMeasure.mut_replace(
                          modules[3].getModuleData().driveVelocityMPerSec,
                          MetersPerSecond));
            },
            subsystem));
  }
}
