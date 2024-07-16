package frc.robot.devices.gyro;

public class GyroFactory {
    private static final GyroFactory INSTANCE = new GyroFactory();

    public static GyroFactory get() {
        return INSTANCE;
    }

    private GyroFactory() {}

    public Gyro create(boolean isReal) {
        if (isReal) return new GyroNavX();
        else return new GyroSim();
    }
}
