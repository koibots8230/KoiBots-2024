package frc.robot.devices;

import edu.wpi.first.units.*;

public class Gyro {
    private Measure<Angle> roll;
    private Measure<Angle> pitch;
    private Measure<Angle> yaw;

    public Gyro() {
        zero();
    }

    public Measure<Angle> getRoll() {
        return roll;
    }

    public Measure<Angle> getPitch() {
        return pitch;
    }

    public Measure<Angle> getYaw() {
        return yaw;
    }

    public void setRoll(Measure<Angle> roll) {
        this.roll = roll;
    }

    public void setPitch(Measure<Angle> pitch) {
        this.pitch = pitch;
    }

    public void setYaw(Measure<Angle> yaw) {
        this.yaw = yaw;
    }

    public void zero() {
        roll = Units.Radians.zero();
        pitch = Units.Radians.zero();
        yaw = Units.Radians.zero();
    }
}
