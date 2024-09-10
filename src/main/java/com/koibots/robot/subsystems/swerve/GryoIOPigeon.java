package com.koibots.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.koibots.robot.Constants.DeviceIDs;

public class GryoIOPigeon implements GyroIO {
    
    private Pigeon2 gyro;

    public GryoIOPigeon() {
        gyro = new Pigeon2(DeviceIDs.PIGEON);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawPosition = gyro.getRotation2d();
        inputs.yawVelocityRadPerSec = gyro.getRate();
    }

    @Override
    public void zeroYaw() {
        gyro.reset();
    }
}
