package frc.robot.devices.motor;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public interface Motor {
    void setVelocity(Measure<Velocity<Angle>> velocity);
    void setPosition(Measure<Angle> position);
    Measure<Velocity<Angle>> getVelocity();
    Measure<Angle> getPosition();
}
