package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**This class allows you extended control over your MotorControlGroup, similar to how the
 * DcMotorEx class allows extended control over a typical DcMotor.*/
public class MotorControlGroupEx extends MotorControlGroup {

    public static DcMotorEx motor1, motor2;

    /**Initialises the variable, and sets internal variables to the two motors to be controlled.
     * @param motor1 the motor to be globally defined as motor #1.
     * @param motor2 the motor to be globally defined as motor #2.*/
    public MotorControlGroupEx(DcMotorEx motor1, DcMotorEx motor2) {
        super(motor1, motor2);

        this.motor1 = motor1;
        this.motor2 = motor2;

    }

    /**Sets the velocity of the two motors. This sets the motors to whatever power is needed
     * to reach the amount of encoder ticks per second specified.
     * @param angularRate the amount of encoder ticks per second to be recorded.*/
    public void setVelocity(double angularRate) {

        motor1.setVelocity(angularRate);
        motor2.setVelocity(angularRate);

    }

    /**Sets the velocity of the two motors. This sets the motors to whatever power is needed
     * to reach the velocity specified in the units specified.
     * @param angularRate the amount of units to be recorded per second.
     * @param unit the unit to record in.*/
    public void setVelocity(double angularRate, AngleUnit unit) {

        motor1.setVelocity(angularRate, unit);
        motor2.setVelocity(angularRate, unit);

    }

    /**Returns a double of the current velocity the motor is reading, in encoder ticks recorded
     * per second.*/
    public double getVelocity() {

        return (motor1.getVelocity() + motor2.getVelocity()) / 2;

    }

    /**Returns a double of the current velocity the motor is reading, in the unit specified
     * per second.
     * @param unit the unit to measure in.*/
    public double getVelocity(AngleUnit unit) {

        return (motor1.getVelocity(unit) + motor2.getVelocity(unit)) / 2;

    }

    /**Sets the target position tolerance. This means once the motors' positions are
     * within the tolerance level, they will return they are in position.
     * @param tolerance the tolerance level to be set.*/
    public void setTargetPositionTolerance(int tolerance) {

        motor1.setTargetPositionTolerance(tolerance);
        motor2.setTargetPositionTolerance(tolerance);

    }

}
