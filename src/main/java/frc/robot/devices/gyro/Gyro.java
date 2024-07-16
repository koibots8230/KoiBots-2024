package frc.robot.devices.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public interface Gyro {
    public Rotation2d getYaw();
    public void reset();
    public void update(Measure<Velocity<Angle>> velocity);
}
