package frc.robot.devices;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import java.util.ArrayList;
import java.util.List;

public class Motor {
    private CANSparkMax motor;
    private RelativeEncoder relativeEncoder;
    private boolean hasAbsoluteEncoder;
    private AbsoluteEncoder absoluteEncoder;
    private Measure<Velocity<Angle>> setpointVelocity = Units.RPM.zero();
    private Measure<Voltage> setpointVoltage = Units.Volts.zero();
    private String id;
    private PIDController PID;
    private Measure<Angle> simPosition = Units.Rotation.zero();
    private static List<Motor> motors = new ArrayList<>();
    private static List<Boolean> motorsUpdated = new ArrayList<>();
    private static int index;

    public Motor(int CANID, String id, double P, double I, double D, boolean hasAbsoluteEncoder) {
        motors.add(this);
        motorsUpdated.add(false);
        index = motors.size() - 1;
        if (motors.size() == motorsUpdated.size()) throw new Error("Check frc.robot.devices.Motor motors and motorsUpdated, not same length");
        this.id = id;
        this.hasAbsoluteEncoder = hasAbsoluteEncoder;
        PID = new PIDController(P, I, D);

        if (Robot.isReal()) {
            motor = new CANSparkMax(CANID, CANSparkLowLevel.MotorType.kBrushless);
            relativeEncoder = motor.getEncoder();
            if (hasAbsoluteEncoder) absoluteEncoder = motor.getAbsoluteEncoder();
        }
    }

    public void update(boolean flag) {
        if (setpointVelocity.in(Units.RPM) != 0 && setpointVoltage.in(Units.Volts) != 0) throw new Error(
                "Check frc.robot.devices.Motor setpoints, both nonzero, unable to determine which setpoint to use");
        if (Robot.isReal()) {
            if (setpointVelocity.in(Units.RPM) != 0) motor.setVoltage(
                    PID.calculate(getVelocity().in(Units.RPM), setpointVelocity.in(Units.RPM)));
            else motor.setVoltage(setpointVoltage.in(Units.Volts));
        } else {
            simPosition = simPosition.plus(
                    Units.Rotations.of(setpointVelocity.divide(50 * 60).in(Units.RPM))).plus(
                    Units.Rotations.of(setpointVoltage.in(Units.Volts) * (4750.0 / 12.0) / (50 * 60)) //bs turning volts into RPM for sim encoder
            );
        }

        if (flag) motorsUpdated.set(index, true);
    }

    public void update() {
        update(true);
    }

    public static void updateAll() {
        for (int index = 0; index < motors.size(); index++) {
            if (!motorsUpdated.get(index)) motors.get(index).update(false);
            motorsUpdated.set(index, false);
        }
    }

    public void updateLoggedValues() {
        if (Robot.isReal()) {
            SmartDashboard.putNumber(id.concat("/Voltage"), motor.getBusVoltage() * motor.getAppliedOutput());
            SmartDashboard.putNumber(id.concat("/Current"), motor.getOutputCurrent());
            SmartDashboard.putNumber(id.concat("/Resistance"), motor.getBusVoltage() *
                    motor.getAppliedOutput() / motor.getOutputCurrent()); //Just for funsies and V=IR
            SmartDashboard.putNumber(id.concat("/Velocity"), relativeEncoder.getVelocity());
        }
        SmartDashboard.putNumber(id.concat("/SetpointVelocity"), setpointVelocity.in(Units.RPM));
        SmartDashboard.putNumber(id.concat("/SetpointVoltage"), setpointVoltage.in(Units.Volts));
    }

    public Measure<Angle> getPosition() {
        if (Robot.isReal()) return Units.Rotations.of(relativeEncoder.getPosition());
        else return simPosition;
    }

    public Measure<Velocity<Angle>> getVelocity() {
        if (Robot.isReal()) return Units.RPM.of(relativeEncoder.getVelocity());
        else return setpointVelocity;
    }

    public void setPosition(Measure<Angle> position) {
        if (Robot.isReal()) relativeEncoder.setPosition(position.in(Units.Rotations));
        else simPosition = position;
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        setpointVelocity = velocity;
        update();
    }

    public void setVoltage(Measure<Voltage> voltage) {
        setpointVoltage = voltage;
        update();
    }
}
