package frc.robot.devices.gyro;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

public class GyroSim implements Gyro{
    private Rotation2d yaw;
    private SimDevice simDevice;
    private SimDouble simYaw;

    public GyroSim() {
        yaw = new Rotation2d();
        simDevice = SimDevice.create("Gyro");
        simYaw = simDevice.createDouble("Yaw (Degrees)", SimDevice.Direction.kOutput, 0);
    }

    @Override
    public Rotation2d getYaw() {
        return yaw;
    }

    @Override
    public void reset() {
        yaw = new Rotation2d();
    }

    public void update(Measure<Velocity<Angle>> velocity) {
        yaw = yaw.plus(Rotation2d.fromRotations(velocity.in(Units.RPM) / 60 / 50));
        simYaw.set(yaw.getDegrees());
    }
}
