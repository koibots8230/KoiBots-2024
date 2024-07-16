package frc.robot.devices.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class GyroNavX implements Gyro {
    private final AHRS gyro;

    public GyroNavX() {
        gyro = new AHRS();

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                reset();
            } catch (Exception ignore) {}
        }).start();
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void reset() {
        gyro.reset();
    }

    @Override
    public void update(Measure<Velocity<Angle>> velocity) {}
}
