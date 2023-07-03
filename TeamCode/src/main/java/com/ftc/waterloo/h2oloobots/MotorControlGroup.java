package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorControlGroup {

    public static DcMotor motor1, motor2;

    public MotorControlGroup(DcMotor motor1, DcMotor motor2) {

        this.motor1 = motor1;
        this.motor2 = motor2;

    }

    public void setDirection(DcMotorSimple.Direction direction) {

        motor1.setDirection(direction);
        motor2.setDirection(direction);

    }

    public void setDirection(DcMotorSimple.Direction motor1Direction,
                             DcMotorSimple.Direction motor2Direction) {

        motor1.setDirection(motor1Direction);
        motor2.setDirection(motor2Direction);

    }

    public DcMotorSimple.Direction[] getDirection() {

        if (motor1.getDirection() == motor2.getDirection()) {

            return new DcMotorSimple.Direction[]{motor1.getDirection()};

        } else {

            return new DcMotorSimple.Direction[]{motor1.getDirection(), motor2.getDirection()};

        }

    }

    public void setPower(double power) {

        motor1.setPower(power);
        motor2.setPower(power);

    }

    public double getPower() {

        return motor1.getPower();

    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {

        motor1.setZeroPowerBehavior(zeroPowerBehavior);
        motor2.setZeroPowerBehavior(zeroPowerBehavior);

    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {

        return motor1.getZeroPowerBehavior();

    }

    public void setTargetPosition(int position) {

        motor1.setTargetPosition(position);
        motor2.setTargetPosition(position);

    }

    public int getTargetPosition() {

        return motor1.getTargetPosition();

    }

    public boolean isBusy() {

        return motor1.isBusy() || motor2.isBusy();

    }

    public int getCurrentPosition() {

        return (motor1.getCurrentPosition() + motor2.getCurrentPosition()) / 2;

    }

    public void setMode(DcMotor.RunMode runMode) {

        motor1.setMode(runMode);
        motor2.setMode(runMode);

    }

}
