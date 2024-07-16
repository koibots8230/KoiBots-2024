// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.autos;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import com.choreo.lib.Choreo;
import com.koibots.robot.Constants.ControlConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.Supplier;

public enum AutoCommands {
    TURN(
        "Turn",
        followChoreoTrajectory("finalboss")
    ),
    X_CALIB(
        "X",
        followChoreoTrajectory("xcalib")
    ),
    Y_CALIB(
        "Y",
        followChoreoTrajectory("ycalib")
    ),
    THETA_CALIB(
        "Theta",
        followChoreoTrajectory("tcalib")
    );

    public final Command command;
    public final String name;

    AutoCommands(String name, Command... commands) {
        this.name = name;
        this.command = new SequentialCommandGroup(commands);
    }

    private static Command followChoreoTrajectory(String trajectory) {
        return Choreo.choreoSwerveCommand(
                Choreo.getTrajectory(trajectory),
                Swerve.get()::getEstimatedPose,
                new PIDController(ControlConstants.X_PID_CONSTANTS.kP, ControlConstants.X_PID_CONSTANTS.kI, ControlConstants.X_PID_CONSTANTS.kD),
                new PIDController(ControlConstants.Y_PID_CONSTANTS.kP, ControlConstants.Y_PID_CONSTANTS.kI, ControlConstants.Y_PID_CONSTANTS.kD),
                new PIDController(ControlConstants.ROTATION_PID_CONSTANTS.kP, ControlConstants.ROTATION_PID_CONSTANTS.kI, ControlConstants.ROTATION_PID_CONSTANTS.kD),
                Swerve.get()::driveRobotRelative,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red)
                            .isPresent();
                },
                Swerve.get());
    }
}