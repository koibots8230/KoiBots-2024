package frc.robot.devices;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import frc.robot.Constants;

public class SwerveModule {
    private final Motor driveMotor;
    private final Motor turnMotor;
    private final PIDController turnPID;
    private SwerveModuleState state;

    public SwerveModule(Motor driveMotor, Motor turnMotor) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;

        turnPID = new PIDController(
                Constants.Drivetrain.Turning.P,
                Constants.Drivetrain.Turning.I,
                Constants.Drivetrain.Turning.D);
    }

    public Measure<Velocity<Angle>> DVToAV (Measure<Velocity<Distance>> velocity) {
        return Units.RotationsPerSecond.of(velocity.in(Units.MetersPerSecond) /
                Constants.Drivetrain.Driving.WHEEL_DIAMETER.in(Units.Meters) * Constants.Drivetrain.Driving.GEARING);
    }

    public Measure<Velocity<Distance>> AVToDV (Measure<Velocity<Angle>> velocity) {
        return Units.MetersPerSecond.of(velocity.in(Units.RotationsPerSecond) *
                Constants.Drivetrain.Driving.WHEEL_DIAMETER.in(Units.Meters) / Constants.Drivetrain.Driving.GEARING);
    }

    public Measure<Angle> getAngle() {
        return Units.Radians.of(turnMotor.getAbsoluteEncoder().getPosition() / (Math.PI * 2));
    }

    public Measure<Velocity<Angle>> getVelocity() {
        return Units.RPM.of(driveMotor.getRelativeEncoder().getVelocity() / (Math.PI * 2));
    }

    public SwerveModuleState getTargetState() {
        return state;
    }

    public SwerveModuleState getRealState() {
        return new SwerveModuleState(AVToDV(getVelocity()).in(Units.MetersPerSecond), new Rotation2d(getAngle()));
    }

    public void setStateNoOptimize(SwerveModuleState state) {
        this.state = state;

        double turnFeedback = turnPID.calculate(getAngle().in(Units.Radians), state.angle.getRadians());
        Measure<Voltage> turnVoltage = Units.Volts.of(
                turnFeedback + Math.signum(turnFeedback) * Constants.Drivetrain.Turning.S);
        turnMotor.setVoltage(turnVoltage);

        driveMotor.setVelocity(DVToAV(Units.MetersPerSecond.of(state.speedMetersPerSecond)));
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, new Rotation2d(getAngle())); // Angle Optimize
        optimized.speedMetersPerSecond *= Math.cos( // Cosine Optimize
                Math.abs(optimized.angle.getRadians() - getAngle().in(Units.Radians)));
        setStateNoOptimize(optimized);
    }
}
