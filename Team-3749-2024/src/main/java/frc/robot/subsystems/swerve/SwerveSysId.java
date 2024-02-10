package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

public class SwerveSysId {

    private SwerveModule[] modules;
    private final MutableMeasure<Voltage> identificationVoltageMeasure = mutable(Volts.of(0));
    private final MutableMeasure<Distance> identificationDistanceMeasure = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> identificaitonVelocityMeasure = mutable(MetersPerSecond.of(0));

    private SysIdRoutine routine = new SysIdRoutine(
            // new SysIdRoutine.Config(),
            new SysIdRoutine.Config(Volts.per(Seconds).of(1), Volts.of(7), Seconds.of(10)),
            new SysIdRoutine.Mechanism(Robot.swerve::identificationDriveConsumer,
                    log -> {
                        // Record a frame for the left motors. Since these share an encoder, we consider
                        // the entire group to be one motor.
                        log.motor("front-left")
                                .voltage(
                                        identificationVoltageMeasure.mut_replace(
                                                modules[0].getModuleData().driveAppliedVolts, Volts))
                                .linearPosition(
                                        identificationDistanceMeasure
                                                .mut_replace(modules[0].getModuleData().drivePositionM, Meters))
                                .linearVelocity(
                                        identificaitonVelocityMeasure.mut_replace(
                                                modules[0].getModuleData().driveVelocityMPerSec,
                                                MetersPerSecond));
                        // Record a frame for the right motors. Since these share an encoder, we
                        // consider
                        // the entire group to be one motor.
                        log.motor("front-right")
                                .voltage(
                                        identificationVoltageMeasure.mut_replace(
                                                modules[1].getModuleData().driveAppliedVolts, Volts))
                                .linearPosition(
                                        identificationDistanceMeasure
                                                .mut_replace(modules[1].getModuleData().drivePositionM, Meters))
                                .linearVelocity(
                                        identificaitonVelocityMeasure.mut_replace(
                                                modules[1].getModuleData().driveVelocityMPerSec,
                                                MetersPerSecond));

                        log.motor("back-left")
                                .voltage(
                                        identificationVoltageMeasure.mut_replace(
                                                modules[2].getModuleData().driveAppliedVolts, Volts))
                                .linearPosition(
                                        identificationDistanceMeasure
                                                .mut_replace(modules[2].getModuleData().drivePositionM, Meters))
                                .linearVelocity(
                                        identificaitonVelocityMeasure.mut_replace(
                                                modules[2].getModuleData().driveVelocityMPerSec,
                                                MetersPerSecond));
                        log.motor("back-right")
                                .voltage(
                                        identificationVoltageMeasure.mut_replace(
                                                modules[3].getModuleData().driveAppliedVolts, Volts))
                                .linearPosition(
                                        identificationDistanceMeasure
                                                .mut_replace(modules[3].getModuleData().drivePositionM, Meters))
                                .linearVelocity(
                                        identificaitonVelocityMeasure.mut_replace(
                                                modules[3].getModuleData().driveVelocityMPerSec,
                                                MetersPerSecond));
                    },
                    Robot.swerve));

    public SwerveSysId(SwerveModule[] modules) {
        this.modules = modules;

    }

    public Command getSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command getSysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public SysIdRoutine getSysIdRoutine() {
        return routine;
    }

}
