package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**This class just makes it easier to visualise the encoders as seperate objects from the motors their ports are associated with.*/
public class Encoder {

    DcMotorEx encoder;

    double countsPerRevolution;
    double wheelDiameter;
    double wheelCircumference;

    public enum MeasurementUnit {

        IN,
        MM

    }

    MeasurementUnit measurementUnit;

    public Encoder(HardwareMap hardwareMap, String deviceName, double countsPerRevolution, double wheelDiameter, MeasurementUnit measurementUnit) {

        this.encoder = (DcMotorEx) hardwareMap.dcMotor.get(deviceName);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.countsPerRevolution = countsPerRevolution;
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.measurementUnit = measurementUnit;

    }

    public void setDirection(DcMotorSimple.Direction direction) {

        encoder.setDirection(direction);

    }

    public void setMode(DcMotor.RunMode mode) {

        encoder.setMode(mode);

    }

    public void setTargetPosition(int position) {

        encoder.setTargetPosition(position);

    }

    public void setTargetPositionTolerance(int tolerance) {

        encoder.setTargetPositionTolerance(tolerance);

    }

    public int getCurrentPosition() {

        return encoder.getCurrentPosition();

    }

    public double getRotations() {

        return this.getCurrentPosition() / countsPerRevolution;

    }

    public double getDistance() {

        return this.getRotations() * this.wheelCircumference;

    }

    public double getDistance(MeasurementUnit measurementUnit) {

        double distance = 0;

        if (measurementUnit != this.measurementUnit) {

            switch (measurementUnit) {

                case IN:
                    distance = this.getDistance() / 25.4;
                    break;

                case MM:
                    distance = this.getDistance() * 25.4;
                    break;

            }

        } else {

            distance = this.getDistance();

        }

        return distance;

    }

}
