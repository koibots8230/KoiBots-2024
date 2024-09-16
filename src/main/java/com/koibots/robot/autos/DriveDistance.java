// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.autos;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.*;

import com.koibots.robot.Constants.AutoConstants;
import com.koibots.robot.Constants.ControlConstants;
import com.koibots.robot.Constants.RobotConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveDistance extends Command {

    private Pose2d endGoal;
    private Rotation2d angle;

    private boolean leaving = false;
    private int note = 100;

    private final PIDController angleAlignmentController;

    public DriveDistance(Pose2d goal) {
        leaving = false;
        note = 100;
        endGoal = goal;

        addRequirements(Swerve.get());

        angleAlignmentController =
                new PIDController(
                        .92,
                        ControlConstants.ANGLE_ALIGNMENT_PID_CONSTANTS.kI,
                        ControlConstants.ANGLE_ALIGNMENT_PID_CONSTANTS.kD);
        SmartDashboard.putData("Angle Alignment Controller", angleAlignmentController);

        angleAlignmentController.enableContinuousInput(0, 2 * Math.PI);
        angleAlignmentController.setTolerance(0.05);
    }

    public DriveDistance(boolean leaving) {
        this.leaving = true;
        endGoal = new Pose2d();
        angle = new Rotation2d();

        angleAlignmentController =
                new PIDController(
                        ControlConstants.ANGLE_ALIGNMENT_PID_CONSTANTS.kP,
                        ControlConstants.ANGLE_ALIGNMENT_PID_CONSTANTS.kI,
                        ControlConstants.ANGLE_ALIGNMENT_PID_CONSTANTS.kD);

        SmartDashboard.putData("Angle Alignment Controller", angleAlignmentController);

        angleAlignmentController.enableContinuousInput(0, 2 * Math.PI);
        angleAlignmentController.setTolerance(0.05);
    }

    public DriveDistance(int note) {
        this.note = note;
        endGoal = new Pose2d();
        angle = new Rotation2d();

        angleAlignmentController =
                new PIDController(
                        ControlConstants.ANGLE_ALIGNMENT_PID_CONSTANTS.kP,
                        ControlConstants.ANGLE_ALIGNMENT_PID_CONSTANTS.kI,
                        ControlConstants.ANGLE_ALIGNMENT_PID_CONSTANTS.kD);

        SmartDashboard.putData("Angle Alignment Controller", angleAlignmentController);

        angleAlignmentController.enableContinuousInput(0, 2 * Math.PI);
        angleAlignmentController.setTolerance(0.05);
    }

    @Override
    public void initialize() {
        if (leaving) {
            endGoal =
                    new Pose2d(
                            Swerve.get().getEstimatedPose().getX() + 2,
                            Swerve.get().getEstimatedPose().getY(),
                            new Rotation2d());
        } else if (note != 100) {
            endGoal =
                    new Pose2d(
                            AutoConstants.NOTE_POSITIONS[note],
                            new Rotation2d(
                                            Swerve.get().getEstimatedPose().getX()
                                                    - AutoConstants.NOTE_POSITIONS[note].getX(),
                                            Swerve.get().getEstimatedPose().getY()
                                                    - AutoConstants.NOTE_POSITIONS[note].getY())
                                    .minus(new Rotation2d(Math.PI)));
            if (endGoal.getRotation().getRadians() < 0.005
                    && endGoal.getRotation().getRadians() > -0.005) {
                endGoal = new Pose2d(endGoal.getTranslation(), new Rotation2d());
            }
        }

        angle =
                new Rotation2d(
                        endGoal.getX() - Swerve.get().getEstimatedPose().getX(),
                        endGoal.getY() - Swerve.get().getEstimatedPose().getY());
    }

    @Override
    public void execute() {
        // if (Math.abs( // Looks complicated, but is just distance calculation to an angle-point
        // line https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        //                 (Math.cos(angle.getRadians() - Math.PI)
        //                                 * (endGoal.getY() -
        // Swerve.get().getEstimatedPose().getY()))
        //                         - (Math.sin(angle.getRadians() - Math.PI)
        //                                 * (endGoal.getX()
        //                                         - Swerve.get().getEstimatedPose().getX())))
        //         > AutoConstants.REPLANNING_THRESHOLD.in(Meters)) {
        //     angle =
        //             new Rotation2d(
        //                     endGoal.getX() - Swerve.get().getEstimatedPose().getX(),
        //                     endGoal.getY() - Swerve.get().getEstimatedPose().getY());
        // }

        double angularVelocity =
                angleAlignmentController.calculate(
                        Swerve.get().getGyroAngle().getRadians(),
                        endGoal.getRotation().getRadians());

        angularVelocity *= angularVelocity * angularVelocity;

        // if (angularVelocity > -0.01 && angularVelocity < 0.01) {
        //     angularVelocity = 0;
        // }

        ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        angle.getCos() * 3,
                        angle.getSin() * 3,
                        angularVelocity * RobotConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
                        Swerve.get().getEstimatedPose().getRotation());

        Swerve.get().driveRobotRelative(speeds);
    }

    @Override
    public boolean isFinished() {
        return ((Swerve.get().getEstimatedPose().getX()
                        <= endGoal.getX() + ControlConstants.ALLOWED_AUTO_ERROR.in(Meters))
                && (Swerve.get().getEstimatedPose().getX()
                        >= endGoal.getX() - ControlConstants.ALLOWED_AUTO_ERROR.in(Meters))
                && (Swerve.get().getEstimatedPose().getY()
                        <= endGoal.getY() + ControlConstants.ALLOWED_AUTO_ERROR.in(Meters))
                && (Swerve.get().getEstimatedPose().getY()
                        >= endGoal.getY() - ControlConstants.ALLOWED_AUTO_ERROR.in(Meters)));
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.get()
                .setModuleStates(
                        new SwerveModuleState[] {
                            new SwerveModuleState(),
                            new SwerveModuleState(),
                            new SwerveModuleState(),
                            new SwerveModuleState()
                        });
    }
}
