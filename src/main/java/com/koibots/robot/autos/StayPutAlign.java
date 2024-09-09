// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.autos;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.koibots.robot.Constants.ControlConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class StayPutAlign extends Command {
    private Pose2d spot;

    private PIDController translationFeedback;
    private PIDController rotationFeedback;

    public StayPutAlign(Pose2d pose) {
        translationFeedback =
                new PIDController(
                        ControlConstants.STAY_PUT_TRANSLATION_PID_CONSTANTS.kP,
                        ControlConstants.STAY_PUT_TRANSLATION_PID_CONSTANTS.kI,
                        ControlConstants.STAY_PUT_TRANSLATION_PID_CONSTANTS.kD);
        rotationFeedback =
                new PIDController(
                        ControlConstants.STAY_PUT_ROTATION_PID_CONSTANTS.kP,
                        ControlConstants.STAY_PUT_ROTATION_PID_CONSTANTS.kI,
                        ControlConstants.STAY_PUT_ROTATION_PID_CONSTANTS.kD);
        rotationFeedback.enableContinuousInput(-Math.PI, Math.PI);

        spot = pose;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Swerve.get()
                .driveRobotRelative(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                MetersPerSecond.of(
                                        translationFeedback.calculate(
                                                Swerve.get().getEstimatedPose().getX(),
                                                spot.getX())),
                                MetersPerSecond.of(
                                        translationFeedback.calculate(
                                                Swerve.get().getEstimatedPose().getY(),
                                                spot.getY())),
                                RadiansPerSecond.of(
                                        rotationFeedback.calculate(
                                                Swerve.get()
                                                        .getEstimatedPose()
                                                        .getRotation()
                                                        .getRadians(),
                                                spot.getRotation().getRadians())),
                                Swerve.get().getGyroAngle()));
    }
}
