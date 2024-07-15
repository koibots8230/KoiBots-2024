// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Swerve;

import com.koibots.robot.Constants.ControlConstants;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;

public class AutoAlign extends Command {

    private Command pathfindingCommand;

    public AutoAlign(Pose2d targetPose, Measure<Distance> rotationDelay) {
        // pathfindingCommand =
        //         new PathfindHolonomic(
        //                 targetPose,
        //                 ControlConstants.PATH_CONSTRAINTS,
        //                 0.0,
        //                 Swerve.get()::getEstimatedPose,
        //                 Swerve.get()::getRelativeSpeeds,
        //                 Swerve.get()::driveRobotRelative,
        //                 ControlConstants.HOLONOMIC_CONFIG,
        //                 rotationDelay.in(Meters),
        //                 Swerve.get());
    }

    public AutoAlign(
            Pose2d targetPose,
            Pose2d secondaryPathStart,
            Measure<Distance> rotationDelay) { // Idk if this is even needed, made it just in case
        List<Translation2d> points =
                PathPlannerPath.bezierFromPoses(secondaryPathStart, targetPose);
        PathPlannerPath path =
                new PathPlannerPath(
                        points,
                        ControlConstants.PATH_CONSTRAINTS,
                        new GoalEndState(0, targetPose.getRotation()));
        path.preventFlipping = true;

        // pathfindingCommand =
        //         new PathfindThenFollowPathHolonomic(
        //                 path,
        //                 ControlConstants.PATH_CONSTRAINTS,
        //                 Swerve.get()::getEstimatedPose,
        //                 Swerve.get()::getRelativeSpeeds,
        //                 Swerve.get()::driveRobotRelative,
        //                 ControlConstants.HOLONOMIC_CONFIG,
        //                 rotationDelay.in(Meters),
        //                 () -> false,
        //                 Swerve.get());
    }

    @Override
    public void initialize() {
        pathfindingCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return pathfindingCommand.isFinished();
    }
}
