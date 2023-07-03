package com.ftc.waterloo.h2oloobots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MotorControlGroupEx extends MotorControlGroup {

    public static DcMotorEx motor1, motor2;

    public MotorControlGroupEx(DcMotorEx motor1, DcMotorEx motor2) {
        super(motor1, motor2);

        this.motor1 = motor1;
        this.motor2 = motor2;

    }

    public void setVelocity(double angularRate) {

        motor1.setVelocity(angularRate);
        motor2.setVelocity(angularRate);

    }

    public void setVelocity(double angularRate, AngleUnit unit) {

        motor1.setVelocity(angularRate, unit);
        motor2.setVelocity(angularRate, unit);

    }

    public double getVelocity() {

        return (motor1.getVelocity() + motor2.getVelocity()) / 2;

    }

    public double getVelocity(AngleUnit unit) {

        return (motor1.getVelocity(unit) + motor2.getVelocity(unit)) / 2;

    }

    public void setTargetPositionTolerance(int tolerance) {

        motor1.setTargetPositionTolerance(tolerance);
        motor2.setTargetPositionTolerance(tolerance);

    }

}
