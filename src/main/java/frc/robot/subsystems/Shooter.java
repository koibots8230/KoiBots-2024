package frc.robot.subsystems;


import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.devices.Motor;

public class Shooter extends SubsystemBase {
    private final Motor shooterMotorTop;
    private final Motor shooterMotorBottom;

    private final static Shooter INSTANCE = new Shooter();

    private Shooter() {
        shooterMotorTop = new Motor(Constants.Motors.ShooterTop);
        shooterMotorBottom = new Motor(Constants.Motors.ShooterBottom);
    }

    public static Shooter getInstance() {
        return INSTANCE;
    }

    public void setVelocity(Measure<Velocity<Angle>> velocityTop, Measure<Velocity<Angle>> velocityBottom) {
        shooterMotorTop.setVelocity(velocityTop);
        shooterMotorBottom.setVelocity(velocityBottom);
        Units.RPM.of(0);
    }
}

