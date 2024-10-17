// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

    public static final double LOOP_PERIOD_SECS = 0.02;

    private final DCMotorSim sim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);

    private Measure<Voltage> appliedVolts = Volts.of(0);

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        sim.update(LOOP_PERIOD_SECS);

        inputs.leftVelocity = sim.getAngularVelocityRPM();
        inputs.rightVelocity = sim.getAngularVelocityRPM();

        inputs.leftCurrent = Amps.of(sim.getCurrentDrawAmps());
        inputs.leftVoltage = appliedVolts;

        inputs.rightCurrent = Amps.of(sim.getCurrentDrawAmps());
        inputs.rightVoltage = appliedVolts;
    }

    @Override
    public void setVoltage(Measure<Voltage> volts) {
        sim.setInputVoltage(volts.in(Volts));
        appliedVolts = volts;
    }
}
