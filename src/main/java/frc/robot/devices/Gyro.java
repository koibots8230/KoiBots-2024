package frc.robot.devices;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Robot;

public class Gyro {
    private final AHRS gyro;
    private Measure<Angle> rollTrim;
    private Measure<Angle> pitchTrim;
    private Measure<Angle> yawTrim;

    public Gyro() {
        if (Robot.isReal()) {
            gyro = new AHRS(SPI.Port.kMXP);
            new Thread(() -> {
                try {
                    Thread.sleep(1000);
                    zero();
                } catch (Exception ignored) {
                }
            }).start();
        } else {
            gyro = null;
        }
    }

    public Measure<Angle> getRoll() {
        return Units.Degrees.of(gyro.getRoll()).plus(rollTrim);
    }

    public Measure<Angle> getPitch() {
        return Units.Degrees.of(gyro.getPitch()).plus(pitchTrim);
    }

    public Measure<Angle> getYaw() {
        return Units.Degrees.of(gyro.getYaw()).plus(yawTrim);
    }

    public Measure<Angle> getRollTrim() {
        return rollTrim;
    }

    public Measure<Angle> getPitchTrim() {
        return pitchTrim;
    }

    public Measure<Angle> getYawTrim() {
        return yawTrim;
    }

    public void setRollTrim(Measure<Angle> rollTrim) {
        this.rollTrim = rollTrim;
    }

    public void setPitchTrim(Measure<Angle> pitchTrim) {
        this.pitchTrim = pitchTrim;
    }

    public void setYawTrim(Measure<Angle> yawTrim) {
        this.yawTrim = yawTrim;
    }

    public void zero() {
        gyro.reset();
    }
}