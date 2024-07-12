// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.DeviceIDs;
import com.koibots.robot.Constants.MotorConstants;
import com.koibots.robot.Constants.RobotConstants;
import com.koibots.robot.Constants.SensorConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;
    private Rotation2d chassisAngularOffset;

    public SwerveModuleIOSparkMax(int driveId, int turnId) {

        driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveMotor.setSmartCurrentLimit(MotorConstants.DRIVE.currentLimit);
        turnMotor.setSmartCurrentLimit(MotorConstants.TURN.currentLimit);

        driveMotor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));
        turnMotor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));

        driveMotor.setInverted(MotorConstants.DRIVE.inverted);
        turnMotor.setInverted(MotorConstants.TURN.inverted);

        driveMotor.setIdleMode(MotorConstants.DRIVE.idleMode);
        turnMotor.setIdleMode(MotorConstants.TURN.idleMode);

        driveMotor.setCANTimeout((int) MotorConstants.CAN_TIMEOUT.in(Millisecond));
        turnMotor.setCANTimeout((int) MotorConstants.CAN_TIMEOUT.in(Millisecond));

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

        driveEncoder.setPositionConversionFactor(
                SensorConstants.DRIVING_ENCODER_POSITION_FACTOR.in(Meters));
        driveEncoder.setVelocityConversionFactor(
                SensorConstants.DRIVING_ENCODER_VELOCITY_FACTOR.in(MetersPerSecond));

        turnEncoder.setInverted(true);

        turnEncoder.setPositionConversionFactor(
                SensorConstants.TURNING_ENCODER_POSITION_FACTOR.in(Radians));
        turnEncoder.setVelocityConversionFactor(
                SensorConstants.TURNING_ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

        if (turnId == DeviceIDs.FRONT_LEFT_TURN) {
            chassisAngularOffset = Rotation2d.fromRadians((3 * Math.PI) / 2);
        } else if (turnId == DeviceIDs.FRONT_RIGHT_TURN) {
            chassisAngularOffset = new Rotation2d(Math.PI);
        } else if (turnId == DeviceIDs.BACK_LEFT_TURN) {
            chassisAngularOffset = Rotation2d.fromRadians(0);
        } else if (turnId == DeviceIDs.BACK_RIGHT_TURN) {
            chassisAngularOffset =
                    Rotation2d.fromRadians(
                            Math.PI / 2); // Rotation2d.fromDegrees((3 * Math.PI) / 2);
        }

        driveEncoder.setPosition(0.0);
        driveEncoder.setAverageDepth(SensorConstants.DRIVE_ENCODER_SAMPLING_DEPTH);
        driveEncoder.setMeasurementPeriod(16);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.drivePosition = Meters.of(driveEncoder.getPosition());
        inputs.driveVelocity = MetersPerSecond.of(driveEncoder.getVelocity());
        inputs.driveAppliedVoltage =
                Volts.of(driveMotor.getBusVoltage()).times(driveMotor.getAppliedOutput());
        inputs.driveCurrent = Amps.of(driveMotor.getOutputCurrent());

        inputs.turnPosition =
                Rotation2d.fromRadians(turnEncoder.getPosition())
                        .plus(chassisAngularOffset)
                        .minus(Rotation2d.fromRadians(Math.PI));

        inputs.turnVelocity = RadiansPerSecond.of(turnEncoder.getVelocity());
        inputs.turnAppliedVoltage =
                Volts.of(turnMotor.getBusVoltage()).times(turnMotor.getAppliedOutput());
        inputs.turnCurrent = Amps.of(turnMotor.getOutputCurrent());
    }

    @Override
    public void setDriveVoltage(Measure<Voltage> voltage) {
        driveMotor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setTurnVoltage(Measure<Voltage> voltage) {
        turnMotor.setVoltage(voltage.in(Volts));
    }
}
