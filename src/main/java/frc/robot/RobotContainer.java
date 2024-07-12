// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Test;


public class RobotContainer
{
    private GenericHID controller;

    public RobotContainer()
    {
        SmartDashboard.putNumber("/Test/1", 123.4);
        configureBindings();
    }

    private void configureBindings() {
        controller = new GenericHID(0);
        Trigger button1 = new Trigger(() -> controller.getRawButton(1));
        button1.onTrue(new InstantCommand(() -> Test.getInstance().setMotor(Units.RPM.of(500))));
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
