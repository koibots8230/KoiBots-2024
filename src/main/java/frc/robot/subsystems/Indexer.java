package frc.robot.subsystems;


import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.devices.Motor;

public class Indexer extends SubsystemBase {
    private final Motor intakeMotor;

    private final static Indexer INSTANCE = new Indexer();

    private Indexer() {
        intakeMotor = new Motor(Constants.Motors.Intake);
    }

    public static Indexer getInstance() {
        return INSTANCE;
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        intakeMotor.setVelocity(velocity);
    }
}

