package frc.robot.subsystems;


import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.devices.Motor;

public class Intake extends SubsystemBase {
    private final static Intake INSTANCE = new Intake();
    private final Motor intakeMotor;

    private Intake() {
        intakeMotor = new Motor(Constants.Motors.Intake);
    }

    public static Intake getInstance() {
        return INSTANCE;
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        intakeMotor.setVelocity(velocity);
    }
}

