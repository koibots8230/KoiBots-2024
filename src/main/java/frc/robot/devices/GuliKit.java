package frc.robot.devices;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GuliKit {
    public GenericHID controller;
    public Trigger a;
    public Trigger b;
    public Trigger x;
    public Trigger y;
    public Trigger leftBumper;
    public Trigger rightBumper;
    public Trigger leftTrigger;
    public Trigger rightTrigger;
    public Trigger start;
    public Trigger select;
    public Trigger leftJSPress;
    public Trigger rightJSPress;

    public GuliKit(int port) {
        controller = new GenericHID(port);
        a = new Trigger(this::getA);
        b = new Trigger(this::getB);
        x = new Trigger(this::getX);
        y = new Trigger(this::getY);
        leftBumper = new Trigger(this::getLeftBumper);
        rightBumper = new Trigger(this::getRightBumper);
        leftTrigger = new Trigger(this::getLeftTrigger);
        rightTrigger = new Trigger(this::getRightTrigger);
        start = new Trigger(this::getStart);
        select = new Trigger(this::getSelect);
        leftJSPress = new Trigger(this::getLeftJSPress);
        rightJSPress = new Trigger(this::getRightJSPress);
    }

    public boolean getA() {return controller.getRawButton(2);}
    public boolean getB() {return controller.getRawButton(1);}
    public boolean getX() {return controller.getRawButton(4);}
    public boolean getY() {return controller.getRawButton(3);}
    public boolean getLeftBumper() {return controller.getRawButton(5);}
    public boolean getRightBumper() {return controller.getRawButton(6);}
    public boolean getLeftTrigger() {return controller.getRawButton(7);}
    public boolean getRightTrigger() {return controller.getRawButton(8);}
    public boolean getStart() {return controller.getRawButton(9);}
    public boolean getSelect() {return controller.getRawButton(10);}
    public boolean getLeftJSPress() {return controller.getRawButton(11);}
    public boolean getRightJSPress() {return controller.getRawButton(12);}
    public double getLeftJSX() {return controller.getRawAxis(0);}
    public double getLeftJSY() {return -controller.getRawAxis(1);}
    public double getRightJSX() {return controller.getRawAxis(2);}
    public double getRightJSY() {return -controller.getRawAxis(3);}

    public void updateDebug() {
        SmartDashboard.putBoolean("Controller/A", getA());
        SmartDashboard.putBoolean("Controller/B", getB());
        SmartDashboard.putBoolean("Controller/X", getX());
        SmartDashboard.putBoolean("Controller/Y", getY());
        SmartDashboard.putBoolean("Controller/LB", getLeftBumper());
        SmartDashboard.putBoolean("Controller/RB", getRightBumper());
        SmartDashboard.putBoolean("Controller/LT", getLeftTrigger());
        SmartDashboard.putBoolean("Controller/RT", getRightTrigger());
        SmartDashboard.putBoolean("Controller/Start", getStart());
        SmartDashboard.putBoolean("Controller/Select", getSelect());
        SmartDashboard.putBoolean("Controller/LJP", getLeftJSPress());
        SmartDashboard.putBoolean("Controller/RJP", getRightJSPress());
        SmartDashboard.putNumber("Controller/LJX", getLeftJSX());
        SmartDashboard.putNumber("Controller/LJY", getLeftJSY());
        SmartDashboard.putNumber("Controller/RJX", getRightJSX());
        SmartDashboard.putNumber("Controller/RJY", getRightJSY());
    }
}
