package frc.robot.devices.motor;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;

public class MotorSim implements Motor{
    private final SimDevice simDevice;
    private final SimDouble simVelocity;
    private final SimDouble simVelocityFactor;
    private final SimDouble simVelocityPreconv;
    private final SimDouble simPosition;
    private final SimDouble simPositionFactor;
    private final SimDouble simPositionPreconv;
    private boolean usePosition;
    private Measure<Velocity<Angle>> velocity;
    private Measure<Angle> position;
    private Measure<Angle> prevPosition;
    private double velocityFactor;
    private double positionFactor;

    public MotorSim(int ID, double P, double I, double D, double FF, double velocityFactor, double positionFactor) {
        velocity = Units.RPM.zero();
        position = Units.Rotations.zero();
        prevPosition = Units.Rotations.zero();
        simDevice = SimDevice.create("Motor [".concat(String.valueOf(ID)).concat("]"));
        simVelocity = simDevice.createDouble("Velocity (Likely RPM)", SimDevice.Direction.kOutput, 0);
        simVelocityFactor = simDevice.createDouble("Velocity Factor", SimDevice.Direction.kOutput, 0);
        simVelocityPreconv = simDevice.createDouble("Velocity Pre-conversion", SimDevice.Direction.kOutput, 0);
        simPosition = simDevice.createDouble("Position (Likely Rotations)", SimDevice.Direction.kOutput, 0);
        simPositionFactor = simDevice.createDouble("Position Factor", SimDevice.Direction.kOutput, 0);
        simPositionPreconv = simDevice.createDouble("Position Pre-conversion", SimDevice.Direction.kOutput, 0);
        this.velocityFactor = velocityFactor;
        this.positionFactor = positionFactor;
    }

    @Override
    public void setVelocityAngle(Measure<Velocity<Angle>> velocity) {
        usePosition = false;
        this.velocity = velocity;
    }

    @Override
    public void setVelocityDistance(Measure<Velocity<Distance>> velocity) {
        usePosition = false;
        this.velocity = Units.RPM.of(velocity.in(Units.MetersPerSecond));
    }

    @Override
    public void setPosition(Measure<Angle> position) {
        usePosition = true;
        this.position = position;
    }

    @Override
    public Measure<Velocity<Angle>> getVelocityAngle() {
        return velocity;
    }

    @Override
    public Measure<Velocity<Distance>> getVelocityDistance() {
        return Units.MetersPerSecond.of(velocity.in(Units.RPM));
    }

    @Override
    public Measure<Angle> getPosition() {
        return position;
    }

    public void update() {
        simVelocityFactor.set(velocityFactor);
        simPositionFactor.set(positionFactor);

        if (!usePosition) {
            simVelocity.set(velocity.in(Units.RPM));
            simVelocityPreconv.set(velocity.divide(velocityFactor).in(Units.RPM));

            position = position.plus(Units.Rotations.of(
                    velocity.in(Units.Rotations.per(Units.Minute)) / 50));

            simPosition.set(position.in(Units.Rotations));
            simPositionPreconv.set(position.divide(positionFactor).in(Units.Rotations));
        }
        else {
            simPosition.set(position.in(Units.Rotations));
            simPositionPreconv.set(position.divide(positionFactor).in(Units.Rotations));

            velocity = position.minus(prevPosition).per(Units.Minute.one().divide(50));

            simVelocity.set(velocity.in(Units.RPM));
            simVelocityPreconv.set(velocity.divide(velocityFactor).in(Units.RPM));
        }

        prevPosition = position;
    }
}
