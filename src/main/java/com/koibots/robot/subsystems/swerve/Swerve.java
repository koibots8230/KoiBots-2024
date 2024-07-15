// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.swerve;

import com.koibots.robot.Constants.ControlConstants;
import com.koibots.robot.Constants.DeviceIDs;
import com.koibots.robot.Constants.RobotConstants;
import com.koibots.robot.Constants.VisionConstants;
import com.koibots.robot.Robot;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    SwerveModule[] swerveModules;
    GyroIO gyro;
    GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    SwerveDrivePoseEstimator odometry;

    public Swerve() {
        if (Robot.isReal()) {
            swerveModules =
                    new SwerveModule[] { // FL-FR-BL-BR
                        new SwerveModule(
                                new SwerveModuleIOSparkMax(
                                        DeviceIDs.FRONT_LEFT_DRIVE, DeviceIDs.FRONT_LEFT_TURN),
                                0),
                        new SwerveModule(
                                new SwerveModuleIOSparkMax(
                                        DeviceIDs.FRONT_RIGHT_DRIVE, DeviceIDs.FRONT_RIGHT_TURN),
                                1),
                        new SwerveModule(
                                new SwerveModuleIOSparkMax(
                                        DeviceIDs.BACK_LEFT_DRIVE, DeviceIDs.BACK_LEFT_TURN),
                                2),
                        new SwerveModule(
                                new SwerveModuleIOSparkMax(
                                        DeviceIDs.BACK_RIGHT_DRIVE, DeviceIDs.BACK_RIGHT_TURN),
                                3),
                    };

            gyro = new GyroIONavX();

        } else {
            swerveModules =
                    new SwerveModule[] {
                        new SwerveModule(new SwerveModuleIOSim(), 0),
                        new SwerveModule(new SwerveModuleIOSim(), 1),
                        new SwerveModule(new SwerveModuleIOSim(), 2),
                        new SwerveModule(new SwerveModuleIOSim(), 3)
                    };

            gyro = new GyroIOSim();
        }

        odometry =
                new SwerveDrivePoseEstimator(
                        ControlConstants.SWERVE_KINEMATICS,
                        gyroInputs.yawPosition,
                        getModulePositions(),
                        new Pose2d());

        try (Notifier odometryUpdater =
                new Notifier(
                        () -> {
                            gyro.updateInputs(gyroInputs);
                            odometry.updateWithTime(
                                    Logger.getRealTimestamp(),
                                    gyroInputs.yawPosition,
                                    getModulePositions());
                        })) {
            odometryUpdater.startPeriodic(1.0 / 200); // Run at 200hz
        }

        odometry.setVisionMeasurementStdDevs(VisionConstants.STDEVS);
    }

    @Override
    public void periodic() {
        gyro.updateInputs(gyroInputs);

        Logger.processInputs("Subsystems/Drive/Gyro", gyroInputs);

        odometry.update(gyroInputs.yawPosition, getModulePositions());

        Logger.recordOutput("Odometry", odometry.getEstimatedPosition());

        swerveModules[0].periodic();
        swerveModules[1].periodic();
        swerveModules[2].periodic();
        swerveModules[3].periodic();

        double[] statesDegrees = new double[8];
        double[] statesRadians = new double[8];
        for (int i = 0; i < 4; i++) {
            statesDegrees[i * 2] = swerveModules[i].getAngle().getDegrees();
            statesDegrees[(i * 2) + 1] = swerveModules[i].getVelocityMetersPerSec();
            statesRadians[i * 2] = swerveModules[i].getAngle().getRadians();
            statesRadians[(i * 2) + 1] = swerveModules[i].getVelocityMetersPerSec();
        }

        Logger.recordOutput("SwerveStates/Measured", statesDegrees);
        Logger.recordOutput("SwerveStates/Measured", statesRadians);

        // field.setRobotPose(getEstimatedPose());
        // SmartDashboard.putData(field);
    }

    public void addVisionMeasurement(Pose2d measurement, Measure<Time> timestamp) {
        System.out.println(measurement);
        odometry.addVisionMeasurement(measurement, timestamp.in(Seconds)); 
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] targetModuleStates =
                ControlConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                targetModuleStates, RobotConstants.MAX_LINEAR_SPEED);

        if (speeds.vxMetersPerSecond == 0.0
                && speeds.vyMetersPerSecond == 0.0
                && speeds.omegaRadiansPerSecond == 0) {
            var currentStates = this.getModuleStates();
            targetModuleStates[0] = new SwerveModuleState(0, currentStates[0].angle);
            targetModuleStates[1] = new SwerveModuleState(0, currentStates[1].angle);
            targetModuleStates[2] = new SwerveModuleState(0, currentStates[2].angle);
            targetModuleStates[3] = new SwerveModuleState(0, currentStates[3].angle);
        }

        this.setModuleStates(targetModuleStates);
    }

    public void driveRobotRelativeByModule(ChassisSpeeds speeds, boolean[] whichModules) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] targetModuleStates =
                ControlConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                targetModuleStates, RobotConstants.MAX_LINEAR_SPEED);

        if (speeds.vxMetersPerSecond == 0.0
                && speeds.vyMetersPerSecond == 0.0
                && speeds.omegaRadiansPerSecond == 0) {
            var currentStates = this.getModuleStates();
            targetModuleStates[0] = new SwerveModuleState(0, currentStates[0].angle);
            targetModuleStates[1] = new SwerveModuleState(0, currentStates[1].angle);
            targetModuleStates[2] = new SwerveModuleState(0, currentStates[2].angle);
            targetModuleStates[3] = new SwerveModuleState(0, currentStates[3].angle);
        }

        for (int a = 0; a < 4; a++) {
            targetModuleStates[a] =
                    whichModules[a] ? targetModuleStates[a] : new SwerveModuleState();
        }

        this.setModuleStates(targetModuleStates);
    }

    public Rotation2d getGyroAngle() {
        return gyroInputs.yawPosition;
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            swerveModules[0].getState(),
            swerveModules[1].getState(),
            swerveModules[2].getState(),
            swerveModules[3].getState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            swerveModules[0].getPosition(),
            swerveModules[1].getPosition(),
            swerveModules[2].getPosition(),
            swerveModules[3].getPosition()
        };
    }

    public ChassisSpeeds getRelativeSpeeds() {
        return ControlConstants.SWERVE_KINEMATICS.toChassisSpeeds(this.getModuleStates());
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(gyroInputs.yawPosition, getModulePositions(), pose);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        Logger.recordOutput(
                "SwerveStates/Setpoints",
                swerveModules[0].setState(states[0]),
                swerveModules[1].setState(states[1]),
                swerveModules[2].setState(states[2]),
                swerveModules[3].setState(states[3]));
    }

    public void stop() {
        Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
        swerveModules[0].stop();
        swerveModules[1].stop();
        swerveModules[2].stop();
        swerveModules[3].stop();
    }

    public void setCross() {
        setModuleStates(
                new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(0)), // Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, new Rotation2d(0)), // Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, new Rotation2d(0)), // Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, new Rotation2d(0)), // Rotation2d.fromDegrees(45))
                });
    }

    public Pose2d getEstimatedPose() {
        return odometry.getEstimatedPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        SwerveModuleState[] measuredStates = getModuleStates();

        builder.addDoubleProperty("Front Left Angle", measuredStates[0].angle::getDegrees, null);
        builder.addDoubleProperty(
                "Front Left Velocity", () -> measuredStates[0].speedMetersPerSecond, null);

        builder.addDoubleProperty(
                "Front Right Angle", () -> measuredStates[1].angle.getDegrees(), null);
        builder.addDoubleProperty(
                "Front Right Velocity", () -> measuredStates[1].speedMetersPerSecond, null);

        builder.addDoubleProperty(
                "Back Left Angle", () -> measuredStates[2].angle.getDegrees(), null);
        builder.addDoubleProperty(
                "Back Left Velocity", () -> measuredStates[2].speedMetersPerSecond, null);

        builder.addDoubleProperty(
                "Back Right Angle", () -> measuredStates[3].angle.getDegrees(), null);
        builder.addDoubleProperty(
                "Back Right Velocity", () -> measuredStates[3].speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> gyroInputs.yawPosition.getDegrees(), null);
    }
}
