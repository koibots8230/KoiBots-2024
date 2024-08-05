// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Intake;

import static com.koibots.robot.subsystems.Subsystems.*;
import static edu.wpi.first.units.Units.RPM;

import com.koibots.robot.Constants.SetpointConstants;
import com.koibots.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeCommand extends SequentialCommandGroup {

    public IntakeCommand() {
        addCommands(
                new ParallelRaceGroup(
                        new StartEndCommand(
                                () -> Intake.get().setVelocity(SetpointConstants.INTAKE_SPEED),
                                () -> Intake.get().setVelocity(RPM.of(0)),
                                Intake.get()),
                        new RunIndexer()),
                new InstantCommand(
                        () ->
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                        new InstantCommand(
                                                                () -> LEDs.get().send_to_rp2040(2)),
                                                        new WaitCommand(1),
                                                        new InstantCommand(
                                                                () ->
                                                                        RobotContainer
                                                                                .rumbleController(
                                                                                        0)),
                                                        new InstantCommand(
                                                                () ->
                                                                        LEDs.get()
                                                                                .send_to_rp2040(
                                                                                        4))),
                                                new SequentialCommandGroup(
                                                        new InstantCommand(
                                                                () ->
                                                                        RobotContainer
                                                                                .rumbleController(
                                                                                        0.5)),
                                                        new WaitCommand(.4),
                                                        new InstantCommand(
                                                                () ->
                                                                        RobotContainer
                                                                                .rumbleController(
                                                                                        0))))
                                        .schedule()));
    }
}
