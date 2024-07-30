// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.ControlConstants;
import com.koibots.robot.Constants.DeviceIDs;
import com.koibots.robot.Constants.MotorConstants;
import com.koibots.robot.Constants.RobotConstants;
import com.koibots.robot.Constants.SensorConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    private final SimpleMotorFeedforward turnFF;

    private final TrapezoidProfile turnProfile;
    private TrapezoidProfile.State goal;

    private TrapezoidProfile.State setpoint;

    private final SparkPIDController turnPID;

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
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 8);

        driveEncoder.setPosition(0.0);
        driveEncoder.setAverageDepth(SensorConstants.DRIVE_ENCODER_SAMPLING_DEPTH);
        driveEncoder.setMeasurementPeriod(16);

        turnPID = turnMotor.getPIDController();
        turnPID.setP(ControlConstants.TURN_PID_CONSTANTS.kP);
        turnPID.setI(ControlConstants.TURN_PID_CONSTANTS.kI);
        turnPID.setD(0);

        turnPID.setFeedbackDevice(turnEncoder);
        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMaxInput(Math.PI);
        turnPID.setPositionPIDWrappingMinInput(-Math.PI);

        turnFF =
                new SimpleMotorFeedforward(
                        ControlConstants.TURN_FEEDFORWARD_CONSTANTS.ks,
                        ControlConstants.TURN_FEEDFORWARD_CONSTANTS.kv);

        turnProfile = new TrapezoidProfile(new Constraints(Math.PI, Math.PI / 2));
        goal = new TrapezoidProfile.State(turnEncoder.getPosition(), 0);
        setpoint = new TrapezoidProfile.State(turnEncoder.getPosition(), 0);
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

        setpoint = turnProfile.calculate(0.02, setpoint, goal);
        inputs.setpoint = goal.position;
        turnPID.setReference(
                goal.position, ControlType.kPosition, 0, turnFF.calculate(setpoint.velocity));
    }

    @Override
    public void setTurnPosition(Rotation2d position) {
        goal =
                new TrapezoidProfile.State(
                        position.getRadians() + chassisAngularOffset.getRadians(), 0);
    }

    @Override
    public void setDriveVoltage(Measure<Voltage> voltage) {
        voltage =
                (chassisAngularOffset.getRadians() == 0
                                || chassisAngularOffset.getRadians() == Math.PI)
                        ? voltage.times(-1)
                        : voltage;
        driveMotor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setTurnVoltage(Measure<Voltage> voltage) {}
}
