// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static edu.wpi.first.units.Units.*;
import static java.lang.StrictMath.PI;

import com.koibots.lib.geometry.Wheel;
import com.koibots.lib.util.FeedforwardConstantsIO;
import com.koibots.lib.util.MotorConstantsIO;
import com.koibots.lib.util.PIDConstantsIO;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.*;
import java.util.Arrays;
import java.util.Hashtable;
import java.util.List;

public class Constants {
    public static class DeviceIDs {
        public static final int BACK_LEFT_TURN = 5;
        public static final int BACK_LEFT_DRIVE = 4;
        public static final int FRONT_RIGHT_DRIVE = 7;
        public static final int FRONT_RIGHT_TURN = 8;
        public static final int BACK_RIGHT_DRIVE = 2;
        public static final int BACK_RIGHT_TURN = 3;
        public static final int FRONT_LEFT_DRIVE = 6;
        public static final int FRONT_LEFT_TURN = 1;
        public static final int INDEXER = 9;
        public static final int SHOOTER_TOP = 11;
        public static final int SHOOTER_BOTTOM = 13;
        public static final int INTAKE = 14;

        public static final int INDEXER_SENSOR = 0;
        public static final int[] TOP_SHOOTER_ENCODER = {1, 2};
        public static final int[] BOTTOM_SHOOTER_ENCODER = {3, 4};
    }

    public static class SensorConstants {
        public static final int ENCODER_SAMPLES_PER_AVERAGE = 100;

        public static final Measure<Angle> TURNING_ENCODER_POSITION_FACTOR =
                Radians.of(2 * Math.PI);
        public static final Measure<Velocity<Angle>> TURNING_ENCODER_VELOCITY_FACTOR =
                RadiansPerSecond.of((2 * Math.PI) / 60.0);

        public static final Measure<Distance> DRIVING_ENCODER_POSITION_FACTOR =
                Inches.of((1.5 * 2 * Math.PI) / RobotConstants.DRIVE_GEAR_RATIO);
        public static final Measure<Velocity<Distance>> DRIVING_ENCODER_VELOCITY_FACTOR =
                MetersPerSecond.of(
                        ((RobotConstants.DRIVE_WHEELS.radius.in(Meters) * 2 * Math.PI)
                                        / RobotConstants.DRIVE_GEAR_RATIO)
                                / 60.0);

        public static final int DRIVE_ENCODER_SAMPLING_DEPTH = 2;

        public static final double SHOOTER_ALLOWED_ERROR = 0.1;
    }

    public static class MotorConstants {
        public static final Measure<Time> CAN_TIMEOUT =
                Milliseconds.of(
                        20); // Default value, but if CAN utilization gets too high, pop it to 0, or
        // bump it up+

        public static final MotorConstantsIO INTAKE =
                new MotorConstantsIO(false, 60, IdleMode.kCoast);

        public static final MotorConstantsIO INDEXER =
                new MotorConstantsIO(true, 60, IdleMode.kBrake);

        public static final MotorConstantsIO TOP_SHOOTER =
                new MotorConstantsIO(true, 60, IdleMode.kCoast);
        public static final MotorConstantsIO BOTTOM_SHOOTER =
                new MotorConstantsIO(true, 60, IdleMode.kCoast);

        public static final MotorConstantsIO DRIVE =
                new MotorConstantsIO(false, 60, IdleMode.kBrake);
        public static final MotorConstantsIO TURN =
                new MotorConstantsIO(false, 30, IdleMode.kBrake);
    }

    public static class ControlConstants {

        // =====================Drive=====================

        public static final double DRIVE_TURN_KS = 0.0;
        public static final PIDConstantsIO TURN_PID_CONSTANTS =
                new PIDConstantsIO(2, 0, 0.1, 35, 0, 0);
        public static final PIDConstantsIO DRIVE_PID_CONSTANTS =
                new PIDConstantsIO(5.5208e-10, 0, 0, 40, 0, 0);
        public static final FeedforwardConstantsIO DRIVE_FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0.11386, 2.6819, 0.16507, 0, 2.65, 0);
        public static final FeedforwardConstantsIO TURN_FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0, 2, 0, 0, 0, 0);

        public static final double DEADBAND = 0.025;

        public static final PIDConstantsIO ANGLE_ALIGNMENT_PID_CONSTANTS =
                new PIDConstantsIO(0.925, 0, 0, 3.5, 0, 0);

        public static final SwerveDriveKinematics SWERVE_KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(
                                RobotConstants.ROBOT_LENGTH.divide(2),
                                RobotConstants.ROBOT_WIDTH.divide(2)), // Front Left
                        new Translation2d(
                                RobotConstants.ROBOT_LENGTH.divide(2),
                                RobotConstants.ROBOT_WIDTH.divide(-2)), // Front Right
                        new Translation2d(
                                RobotConstants.ROBOT_LENGTH.divide(-2),
                                RobotConstants.ROBOT_WIDTH.divide(2)), // Back Left
                        new Translation2d(
                                RobotConstants.ROBOT_LENGTH.divide(-2),
                                RobotConstants.ROBOT_WIDTH.divide(-2)) // Back Right
                        );

        // =====================Shooter=====================

        public static final FeedforwardConstantsIO TOP_SHOOTER_FEEEDFORWARD =
                new FeedforwardConstantsIO(0, 0.000178, 0, .0021);
        public static final FeedforwardConstantsIO BOTTOM_SHOOTER_FEEDFORWARD =
                new FeedforwardConstantsIO(0, 0.000182, 0, .0021);
        public static final PIDConstantsIO SHOOTER_FEEDBACK_CONSTANTS =
                new PIDConstantsIO(0.000245, 0, 0, .023, 0, 0);

        // =====================Intake=====================

        public static final PIDConstantsIO INTAKE_FEEDBACK_CONSTANTS =
                new PIDConstantsIO(0.01, 0, 0, .6, 0, 0);
        public static final FeedforwardConstantsIO INTAKE_FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0, 10.25, 0, .0022);

        // =====================Indexer=====================

        public static final PIDConstantsIO INDEXER_FEEDBACK_CONSTANTS =
                new PIDConstantsIO(0.05, 0, 0, .36, 0, 0);
        public static final FeedforwardConstantsIO INDEXER_FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0, 2.4, 0, .0021);

        // =====================Pathplanner=====================

        public static final PIDConstantsIO TRANSLATION_PID_CONSTANTS = new PIDConstantsIO(4, 0, 0, 0.0005, 0, 0);
        public static final PIDConstantsIO ROTATION_PID_CONSTANTS = new PIDConstantsIO(5, 0, 0, 0.0005, 0, 0);

        public static final PIDConstantsIO X_PID_CONSTANTS = new PIDConstantsIO(1.65, 0, 0, 0, 0, 0);
        public static final PIDConstantsIO Y_PID_CONSTANTS = new PIDConstantsIO(1.5, 0, 0, 0, 0, 0);
        // public static final PIDConstantsIO ROTATION_PID_CONSTANTS = new PIDConstantsIO(1.35, 0, 0, 0.0005, 0, 0);

        public static final Measure<Distance> REPLANNING_ERROR_THRESHOLD = Inches.of(6);
        public static final Measure<Distance> REPLANNING_ERROR_SPIKE_THRESHOLD = Inches.of(4);

        public static final PathConstraints PATH_CONSTRAINTS =
                new PathConstraints(
                        RobotConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                        RobotConstants.MAX_LINEAR_ACCELERATION.in(MetersPerSecondPerSecond),
                        RobotConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
                        RobotConstants.MAX_ANGULAR_ACCELERATION.in(RadiansPerSecond.per(Second)));

        public static final HolonomicPathFollowerConfig HOLONOMIC_CONFIG =
                new HolonomicPathFollowerConfig(
                        TRANSLATION_PID_CONSTANTS,
                        ROTATION_PID_CONSTANTS,
                        4,
                        Math.sqrt(
                                Math.pow(RobotConstants.ROBOT_LENGTH.in(Meters), 2)
                                        + Math.pow(RobotConstants.ROBOT_WIDTH.in(Meters), 2)),
                        new ReplanningConfig(
                                false,
                                false,
                                REPLANNING_ERROR_THRESHOLD.in(Meters),
                                REPLANNING_ERROR_SPIKE_THRESHOLD.in(Meters)));

        public static final Measure<Distance> ALLOWED_AUTO_ERROR = Inches.of(10);
    }

    public static final class SetpointConstants {
        public static final Measure<Velocity<Angle>> INTAKE_SPEED = RPM.of(600);
        public static final Measure<Velocity<Angle>> INTAKE_REVERSE_SPEED = RPM.of(-600);

        public static final Measure<Velocity<Angle>> SHOOTER_INDEXER_SPEED = RPM.of(3000);
        public static final Measure<Velocity<Angle>> INTAKE_INDEXER_SPEED = RPM.of(600);

        public enum SHOOTER_SPEEDS {
            SPEAKER(Arrays.asList(RPM.of(3850), RPM.of(3850))),
            AMP(Arrays.asList(RPM.of(500), RPM.of(2000))),
            INTAKE(Arrays.asList(RPM.of(-800), RPM.of(-602))),
            REVERSE(Arrays.asList(RPM.of(-400), RPM.of(-400))),
            IDLE(Arrays.asList(RPM.of(500), RPM.of(500)));

            public final Measure<Velocity<Angle>> topSpeed;
            public final Measure<Velocity<Angle>> bottomSpeed;

            SHOOTER_SPEEDS(List<Measure<Velocity<Angle>>> speeds) {
                this.topSpeed = speeds.get(0);
                this.bottomSpeed = speeds.get(1);
            }
        }
    }

    public static final class RobotConstants {
        public static final Measure<Voltage> NOMINAL_VOLTAGE = Volts.of(12);

        public static final Wheel INTAKE_WHEELS = new Wheel(Inches.of(1.5));

        public static final Wheel DRIVE_WHEELS = new Wheel(Inches.of(1.5));
        private static final Measure<Distance> ROBOT_WIDTH = Inches.of(21.375);
        private static final Measure<Distance> ROBOT_LENGTH = Inches.of(21.375);

        public static final Measure<Velocity<Distance>> MAX_LINEAR_SPEED =
                MetersPerSecond.of(4.125);
        public static final Measure<Velocity<Angle>> MAX_ANGULAR_VELOCITY =
                RadiansPerSecond.of(2 * PI);
        public static final Measure<Velocity<Velocity<Distance>>> MAX_LINEAR_ACCELERATION =
                MetersPerSecondPerSecond.of(2.25);
        public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCELERATION =
                RadiansPerSecond.of(4 * Math.PI).per(Second);

        private static final int DRIVING_PINION_TEETH = 13;
        public static final double DRIVE_GEAR_RATIO = (45.0 * 22) / (DRIVING_PINION_TEETH * 15);
        public static final double TURN_GEAR_RATIO = (62.0 / 14) * 12;
    }

    public static class AlignConstants {
        public static final Pose2d AMP_POSITION = new Pose2d();

        public static final Translation2d ALLOWED_DISTANCE_FROM_AMP = new Translation2d(2, 2);

        public static final List<Measure<Distance>> SHOOT_DISTANCES_METERS =
                Arrays.asList(Meters.of(4.5));
        public static final Pose2d SPEAKER_POSITION = new Pose2d();

        public static final Translation2d ALLOWED_DISTANCE_FROM_SHOOT = new Translation2d(2, 2);

        public static final Translation2d ALLOWED_DISTANCE_FROM_NOTE = new Translation2d(2, 2);
    }

    public static class VisionConstants {
        public static final int ACTIVE_CAMERAS = 3;

        public static final Pose2d[] CAMERA_POSITIONS = {
            new Pose2d(-0.26035, -0.1524, new Rotation2d(Math.toRadians(90))),
            new Pose2d(0.0254, -0.1651, new Rotation2d(Math.toRadians(180))),
            new Pose2d(0.26035, -0.1524, new Rotation2d(Math.toRadians(270)))
        }; // x is left, y is forward, counterclockwise on rotation

        public static final String[][] TOPIC_NAMES = {
            {"Cam1Tvec", "Cam1Rvec", "Cam1Ids"},
            {"Cam2Tvec", "Cam2Rvec", "Cam2Ids"},
            {"Cam3Tvec", "Cam3Rvec", "Cam3Ids"}
        };

        public static final double[] VECTOR_DEFAULT_VALUE = {0};
        public static final int ID_DEFAULT_VALUE = 0;

        public static final Measure<Distance> MAX_MEASUREMENT_DIFFERENCE = Meters.of(1);

        public static final Matrix<N3, N1> STDEVS = VecBuilder.fill(2,2, 50 * Math.PI);
    }

    public static class AutoConstants {
        public static Hashtable<String, Pose2d> STARTING_POSITIONS =
                new Hashtable<>() {
                    {
                        put("Subwoofer - Left", new Pose2d(0.668, 6.72, new Rotation2d(PI / 3)));
                        put("Subwoofer - Front", new Pose2d(1.374775, 5.553456, new Rotation2d()));
                        put("Subwoofer - Right", new Pose2d(0.668, 4.39, new Rotation2d(-PI / 3)));
                    }
                };

        public static Pose2d[] SCORING_POSITIONS = {
            new Pose2d(0.668, 6.72, new Rotation2d(PI / 3)),
            new Pose2d(1.374775, 5.553456, new Rotation2d()),
            new Pose2d(0.668, 4.39, new Rotation2d(-PI / 3))
        };

        public static Translation2d[] NOTE_POSITIONS = {
            new Translation2d(2.8956, 7.001256),
            new Translation2d(2.8956, 5.553456),
            new Translation2d(2.8956, 4.105656)
        };

        public static final Measure<Distance> REPLANNING_THRESHOLD = Inches.of(6);

        public static final boolean IS_RED = false;
    }
}
