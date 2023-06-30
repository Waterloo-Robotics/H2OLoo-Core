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

    // This declares the variables for a color sensor, a distance sensor, a touch sensor, a motor, and a servo.
    public static ColorSensor color;
    public static DistanceSensor distance;
    public static TouchSensor touch;
    public static DcMotor motor;
    public static Servo servo;

    // This declares global variables to make our lives easier, we set these when we initialise the file.
    TelemetryControl telemetryControl;
    HardwareMap hardwareMap;

    public AttachmentControl(HardwareMap hardwareMap, TelemetryControl telemetryControl) {

        this.hardwareMap = hardwareMap;
        this.telemetryControl = telemetryControl;

//        color = hardwareMap.colorSensor.get("color");
//        distance = hardwareMap.get(DistanceSensor.class, "distance");
        touch = hardwareMap.touchSensor.get("touch");
        motor = hardwareMap.dcMotor.get("motor");
        servo = hardwareMap.servo.get("servo");

    }



    public void attachmentTelemetry() {

        telemetryControl.addData("Color Sensor - Red", color.red());
        telemetryControl.addData("Color Sensor - Green", color.green());
        telemetryControl.addData("Color Sensor - Blue", color.blue());
        telemetryControl.addData("Color Sensor - Distance (Inches)", ((DistanceSensor) color).getDistance(DistanceUnit.INCH));
        telemetryControl.addData("Distance Sensor - Distance (Inches)", distance.getDistance(DistanceUnit.INCH));
        telemetryControl.addData("Touch Sensor Touched", touch.isPressed());

    }

}
