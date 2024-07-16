package frc.robot.devices.motor;

import edu.wpi.first.units.*;

public interface Motor {
    void setVelocityAngle(Measure<Velocity<Angle>> velocity);
    void setVelocityDistance(Measure<Velocity<Distance>> velocity);
    void setPosition(Measure<Angle> position);
    Measure<Velocity<Angle>> getVelocityAngle();
    Measure<Velocity<Distance>> getVelocityDistance();
    Measure<Angle> getPosition();
}
