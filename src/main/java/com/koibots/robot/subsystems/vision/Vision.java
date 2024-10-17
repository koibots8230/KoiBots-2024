// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.vision;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Microseconds;

import com.koibots.robot.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.networktables.TimestampedInteger;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

    private final DoubleArraySubscriber[][] vecSubscribers;
    private final IntegerSubscriber[] idSubscribers;

    private AprilTagFieldLayout layout;

    public Vision() {
        vecSubscribers = new DoubleArraySubscriber[VisionConstants.ACTIVE_CAMERAS][2];
        idSubscribers = new IntegerSubscriber[VisionConstants.ACTIVE_CAMERAS];
        NetworkTable table = NetworkTableInstance.getDefault().getTable("fisheye");
        for (int a = 0; a < VisionConstants.ACTIVE_CAMERAS; a++) {
            IntegerTopic topic = table.getIntegerTopic(VisionConstants.TOPIC_NAMES[a][2]);
            topic.setCached(true);
            idSubscribers[a] =
                    topic.subscribe(
                            VisionConstants.ID_DEFAULT_VALUE,
                            PubSubOption.pollStorage(10),
                            PubSubOption.sendAll(true),
                            PubSubOption.keepDuplicates(true));

            for (int b = 0; b < 2; b++) {
                DoubleArrayTopic topic2 =
                        table.getDoubleArrayTopic(VisionConstants.TOPIC_NAMES[a][b]);
                topic2.setCached(true);
                vecSubscribers[a][b] =
                        topic2.subscribe(
                                VisionConstants.VECTOR_DEFAULT_VALUE,
                                PubSubOption.pollStorage(10),
                                PubSubOption.sendAll(true),
                                PubSubOption.keepDuplicates(true));
            }
        }
        try {
            layout =
                    new AprilTagFieldLayout(
                            Path.of(
                                    Filesystem.getDeployDirectory().getPath(),
                                    "apriltag",
                                    "2024-crescendo.json"));
        } catch (IOException e) {
            System.err.println("ERROR: Could not find apriltag field layout!");
        }
    }

    private Pose2d translateToFieldPose(
            double[] translation, double[] rotation, int tagId, int camera) {
        int count = 0;
        Matrix<N3, N3> rotMatrix = new Matrix<>(Nat.N3(), Nat.N3());
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) {
                rotMatrix.set(a, b, rotation[count]);
                count++;
            }
        }
        
        double hypotenuse = Math.hypot(translation[0], translation[2]);

        double hypangle =
                layout.getTagPose(tagId).get().getRotation().toRotation2d().getRadians()
                        - Math.atan(translation[0] / translation[2]);

        Pose2d camPose =
                new Pose2d(
                        layout.getTagPose(tagId).get().getX() + (hypotenuse * Math.cos(hypangle)),
                        layout.getTagPose(tagId).get().getY() + (hypotenuse * Math.sin(hypangle)),
                        new Rotation2d());

        Rotation2d angle =
                new Rotation2d(
                        Math.PI
                                + VisionConstants.CAMERA_POSITIONS[camera]
                                        .getRotation()
                                        .getRadians()
                                + layout.getTagPose(tagId).get().getRotation().getZ()
                                + Math.atan2(
                                        -rotMatrix.get(2, 0),
                                        Math.sqrt(
                                                Math.pow(rotMatrix.get(2, 1), 2)
                                                        + Math.pow(rotMatrix.get(2, 2), 2))));

        return new Pose2d(
                camPose.getX()
                        + ((-VisionConstants.CAMERA_POSITIONS[camera].getX()
                                        * Math.cos(Swerve.get().getGyroAngle().getRadians()))
                                + (VisionConstants.CAMERA_POSITIONS[camera].getY()
                                        * Math.sin(Swerve.get().getGyroAngle().getRadians()))),
                camPose.getY()
                        + ((-VisionConstants.CAMERA_POSITIONS[camera].getY()
                                        * Math.cos(Swerve.get().getGyroAngle().getRadians()))
                                - (VisionConstants.CAMERA_POSITIONS[camera].getX()
                                        * Math.sin(Swerve.get().getGyroAngle().getRadians()))),
                angle);
    }

    @Override
    public void periodic() {
        for (int a = 0; a < VisionConstants.ACTIVE_CAMERAS; a++) {
            TimestampedDoubleArray[] tvec = vecSubscribers[a][0].readQueue();
            TimestampedDoubleArray[] rvec = vecSubscribers[a][1].readQueue();
            TimestampedInteger[] ids = idSubscribers[a].readQueue();
            if (tvec.length == 0 || rvec.length == 0 || ids.length == 0) {
                continue;
            } else if (tvec[0].value.length == 1) {
                 System.err.println("Too short2");
                continue;
            }
            while (!(tvec.length == rvec.length && rvec.length == ids.length)) {
                if (tvec.length > rvec.length || tvec.length > ids.length) {
                    tvec = Arrays.copyOf(tvec, tvec.length - 1);
                }
                if (rvec.length > tvec.length || rvec.length > ids.length) {
                    rvec = Arrays.copyOf(rvec, rvec.length - 1);
                }
                if (ids.length > rvec.length || ids.length > tvec.length) {
                    ids = Arrays.copyOf(ids, ids.length - 1);
                }
            }
            for (int b = 0; b < ids.length; b++) {
                if (ids[b].value != 0
                        && tvec[b].timestamp == rvec[b].timestamp
                        && rvec[b].timestamp == ids[b].timestamp) {
                    Pose2d pose =
                            translateToFieldPose(
                                    tvec[b].value, rvec[b].value, (int) ids[b].value, a);
                    System.out.println(pose);
                    // spotless:off
                    if (pose.getY() > 0
                            && pose.getY() < layout.getFieldWidth()
                            && pose.getX() > 0
                            && pose.getX() < layout.getFieldLength()
                            && ((RobotState.isDisabled()) ? true
                                    : Math.sqrt(
                                        Math.pow(pose.getX() - Swerve.get().getEstimatedPose().getX(), 2)
                                            + Math.pow(pose.getY() - Swerve.get().getEstimatedPose().getY(), 2))
                                    < VisionConstants.MAX_MEASUREMENT_DIFFERENCE.in(Meters))) { //TODO: test this with robot movement (is it too low?)
                        // spotless:on
                        Logger.recordOutput("Raw Vision", pose);
                        Swerve.get()
                                .addVisionMeasurement(
                                        pose,
                                        Microseconds.of(ids[b].serverTime),
                                        VecBuilder.fill(
                                                Math.pow(
                                                                1
                                                                        + Math.hypot(
                                                                                tvec[b].value[0],
                                                                                tvec[b].value[2]),
                                                                VisionConstants
                                                                        .TRANSLATION_STDEV_ORDER)
                                                        * VisionConstants.TRANSLATION_STDEV_SCALAR,
                                                Math.pow(
                                                                1
                                                                        + Math.hypot(
                                                                                tvec[b].value[0],
                                                                                tvec[b].value[2]),
                                                                VisionConstants
                                                                        .TRANSLATION_STDEV_ORDER)
                                                        * VisionConstants.TRANSLATION_STDEV_SCALAR,
                                                VisionConstants.ROTATION_STDEV));
                    }
                }
            }
        }
    }
}
