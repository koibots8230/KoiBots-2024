// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ControlConstants;
import com.koibots.robot.Constants.RobotConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private final PIDController driveFeedback;
    private final SimpleMotorFeedforward driveFeedforward;

    private final PIDController turnFeedback;

    private Measure<Velocity<Distance>> driveSetpoint;
    private Rotation2d turnSetpoint;

    public SwerveModuleIOSim() {
        driveSim = new DCMotorSim(DCMotor.getNEO(1), RobotConstants.DRIVE_GEAR_RATIO, 0.025);
        turnSim = new DCMotorSim(DCMotor.getNEO(1), RobotConstants.TURN_GEAR_RATIO, 0.004);

        driveFeedback =
                new PIDController(
                        ControlConstants.DRIVE_PID_CONSTANTS.kP,
                        ControlConstants.DRIVE_PID_CONSTANTS.kI,
                        ControlConstants.DRIVE_PID_CONSTANTS.kD);
        driveFeedforward =
                new SimpleMotorFeedforward(
                        ControlConstants.DRIVE_FEEDFORWARD_CONSTANTS.ks,
                        ControlConstants.DRIVE_FEEDFORWARD_CONSTANTS.kv);

        turnFeedback =
                new PIDController(
                        ControlConstants.TURN_PID_CONSTANTS.kP,
                        ControlConstants.TURN_PID_CONSTANTS.kI,
                        ControlConstants.TURN_PID_CONSTANTS.kD);

        driveSetpoint = MetersPerSecond.of(0);
        turnSetpoint = new Rotation2d();
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveSim.update(LOOP_PERIOD_SECS);
        turnSim.update(LOOP_PERIOD_SECS);

        inputs.drivePosition =
                RobotConstants.DRIVE_WHEELS.radius.times(driveSim.getAngularPositionRad());
        inputs.driveVelocity =
                RobotConstants.DRIVE_WHEELS
                        .radius
                        .times(driveSim.getAngularVelocityRadPerSec())
                        .per(Second);
        inputs.driveCurrent = Amps.of(driveSim.getCurrentDrawAmps());

        inputs.driveAppliedVoltage =
                Volts.of(
                        driveFeedback.calculate(
                                        inputs.driveVelocity.in(MetersPerSecond),
                                        driveSetpoint.in(MetersPerSecond))
                                + driveFeedforward.calculate(driveSetpoint.in(MetersPerSecond)));
        driveSim.setInputVoltage(inputs.driveAppliedVoltage.in(Volts));

        inputs.turnPosition =
                new Rotation2d(turnSim.getAngularPositionRad() % (2 * Math.PI))
                        .minus(Rotation2d.fromRadians(Math.PI));
        inputs.turnVelocity = RadiansPerSecond.of(turnSim.getAngularVelocityRadPerSec());
        inputs.turnCurrent = Amps.of(turnSim.getCurrentDrawAmps());

        inputs.turnAppliedVoltage =
                Volts.of(
                        turnFeedback.calculate(
                                inputs.turnPosition.getRadians(), turnSetpoint.getRadians()));
        turnSim.setInputVoltage(inputs.turnAppliedVoltage.in(Volts));
    }

    @Override
    public void setDriveVelocity(Measure<Velocity<Distance>> velocity) {
        driveSetpoint = velocity;
    }

    @Override
    public void setTurnPosition(Rotation2d position) {
        turnSetpoint = position;
    }

    @Override
    public Rotation2d getTurnRawPosition() {
        return new Rotation2d(turnSim.getAngularPositionRad() % (2 * Math.PI));
    }
}
