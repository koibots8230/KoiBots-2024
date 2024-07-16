package frc.robot.devices.motor;

import java.util.ArrayList;
import java.util.List;

public class MotorFactory {
    private final List<MotorSim> motorSims;

    private static final MotorFactory INSTANCE = new MotorFactory();

    public static MotorFactory get() {
        return INSTANCE;
    }

    private MotorFactory() {
        motorSims = new ArrayList<>();
    }

    public Motor create(int ID, double P, double I, double D, double FF,
                        double velocityFactor, double positionFactor, boolean isReal) {
        if (isReal) {
            return new MotorCANSparkMax(ID, P, I, D, FF, velocityFactor, positionFactor);
        } else {
            MotorSim motorSim = new MotorSim(ID, P, I, D, FF, velocityFactor, positionFactor);
            motorSims.add(motorSim);
            return motorSim;
        }
    }

    public void updateSims() {
        for (MotorSim motorSim : motorSims) motorSim.update();
    }
}
