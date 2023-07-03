package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AttachmentControl {

    // This declares global variables to make our lives easier, we set these when we initialise the file.
    TelemetryControl telemetryControl;
    HardwareMap hardwareMap;

    public AttachmentControl(HardwareMap hardwareMap, TelemetryControl telemetryControl) {

        this.hardwareMap = hardwareMap;
        this.telemetryControl = telemetryControl;

    }

    public void attachmentTelemetry() {



    }

}
