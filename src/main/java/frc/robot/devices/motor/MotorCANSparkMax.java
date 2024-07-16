package frc.robot.devices.motor;

import com.revrobotics.*;
import edu.wpi.first.units.*;

public class MotorCANSparkMax implements Motor {
    private final CANSparkMax canSparkMax;
    private final SparkPIDController controller;
    private final RelativeEncoder encoder;

    public MotorCANSparkMax(
            int ID, double P, double I, double D, double FF, double velocityFactor, double positionFactor) {
        canSparkMax = new CANSparkMax(ID, CANSparkLowLevel.MotorType.kBrushless);
        encoder = canSparkMax.getEncoder();
        encoder.setVelocityConversionFactor(velocityFactor);
        encoder.setPositionConversionFactor(positionFactor);
        controller = canSparkMax.getPIDController();
        controller.setP(P);
        controller.setI(I);
        controller.setD(D);
        controller.setFF(FF);
    }

    @Override
    public void setVelocityAngle(Measure<Velocity<Angle>> velocity) {
        controller.setReference(velocity.in(Units.RPM), CANSparkBase.ControlType.kVelocity);

    }

    @Override
    public void setVelocityDistance(Measure<Velocity<Distance>> velocity) {
        controller.setReference(velocity.in(Units.MetersPerSecond), CANSparkBase.ControlType.kVelocity);
    }

    @Override
    public void setPosition(Measure<Angle> position) {
        controller.setReference(position.in(Units.Rotations), CANSparkBase.ControlType.kPosition);
    }

    @Override
    public Measure<Velocity<Angle>> getVelocityAngle() {
        return Units.RPM.of(encoder.getVelocity());
    }

    @Override
    public Measure<Velocity<Distance>> getVelocityDistance() {
        return Units.MetersPerSecond.of(encoder.getVelocity());
    }

    @Override
    public Measure<Angle> getPosition() {
        return Units.Rotations.of(encoder.getPosition());
    }
}
