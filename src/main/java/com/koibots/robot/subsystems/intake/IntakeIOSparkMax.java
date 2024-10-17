// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.DeviceIDs;
import com.koibots.robot.Constants.MotorConstants;
import com.koibots.robot.Constants.RobotConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.*;

public class IntakeIOSparkMax implements IntakeIO {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder encoder;

    public IntakeIOSparkMax() {

        leftMotor = new CANSparkMax(DeviceIDs.LEFT_INTAKE, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();

        leftMotor.setSmartCurrentLimit(MotorConstants.INTAKE.currentLimit);
        leftMotor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));

        leftMotor.setInverted(MotorConstants.INTAKE.inverted);

        leftMotor.setIdleMode(MotorConstants.INTAKE.idleMode);

        leftMotor.setCANTimeout((int) MotorConstants.CAN_TIMEOUT.in(Milliseconds));

        rightMotor = new CANSparkMax(DeviceIDs.RIGHT_INTAKE, MotorType.kBrushless);

        rightMotor.setIdleMode(MotorConstants.INTAKE.idleMode);

        rightMotor.setCANTimeout((int) MotorConstants.CAN_TIMEOUT.in(Milliseconds));

        rightMotor.follow(leftMotor, true);

        encoder = leftMotor.getEncoder();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.leftVelocity = encoder.getVelocity();

        inputs.leftCurrent = Amps.of(leftMotor.getOutputCurrent());
        inputs.leftVoltage = Volts.of(leftMotor.getBusVoltage()).times(leftMotor.getAppliedOutput());

        inputs.rightVelocity = encoder.getVelocity();

        inputs.rightCurrent = Amps.of(rightMotor.getOutputCurrent());
        inputs.rightVoltage = Volts.of(rightMotor.getBusVoltage()).times(rightMotor.getAppliedOutput());
    }

    @Override
    public void setVoltage(Measure<Voltage> volts) {
        leftMotor.setVoltage(volts.in(Volts));
        rightMotor.setVoltage(volts.in(Volts));
    }
}
