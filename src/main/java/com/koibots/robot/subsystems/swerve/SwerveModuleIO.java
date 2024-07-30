// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleInputs {
        public Measure<Distance> drivePosition = Meters.of(0);
        public Measure<Velocity<Distance>> driveVelocity = MetersPerSecond.of(0);
        public Measure<Voltage> driveAppliedVoltage = Volts.of(0);
        public Measure<Current> driveCurrent = Amps.of(0);

        public Rotation2d turnPosition = new Rotation2d();
        public Measure<Velocity<Angle>> turnVelocity = RadiansPerSecond.of(0);
        public Measure<Voltage> turnAppliedVoltage = Volts.of(0);
        public Measure<Current> turnCurrent = Amps.of(0);
        public double setpoint = 0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(SwerveModuleInputs inputs) {}

    default void setTurnPosition(Rotation2d position) {}

    /** Run the drive motor at the specified voltage. */
    default void setDriveVoltage(Measure<Voltage> volts) {}

    /** Run the turn motor at the specified voltage. */
    default void setTurnVoltage(Measure<Voltage> volts) {}
}
