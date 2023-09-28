package com.ftc.waterloo.h2oloobots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

public class TelemetryControl {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    Telemetry telemetry;

    double fldir = 0;
    double frdir = 0;
    double bldir = 0;
    double brdir = 0;

    double leftDir = 0;
    double rightDir = 0;

    /**The TelemetryControl class converts typical telemetry to also send to the FTC Dashboard.
     * @param telemetry the local telemetry from the OpMode's runOpMode() void.*/
    public TelemetryControl(Telemetry telemetry) {

        this.telemetry = telemetry;
        packet.addLine("Robot Initialised");
        telemetry.addLine("Robot Initialised");

    }

    /**Adds data to the telemetry, with a caption and a value (format: "caption: value")
     * @param caption the caption to be displayed next to the value
     * @param value the value to be displayed*/
    public void addData(String caption, Object value) {

        telemetry.addData(caption, value);
        packet.put(caption, value);

    }

    /**Adds a line of text to the telemetry.
     * @param line the text to be displayed in the line*/
    public void addLine(String line) {

        telemetry.addLine(line);
        packet.clearLines();
        packet.addLine(line);

    }

    /**Updates telemetry to tell what direction the drivebase is moving.
     * @param flpower the power for the front left motor
     * @param frpower the power for the front right motor*/
    public void motorTelemetryUpdate(double flpower, double frpower, double blpower, double brpower) {

        fldir = Math.signum(flpower);
        frdir = Math.signum(frpower);
        bldir = Math.signum(blpower);
        brdir = Math.signum(brpower);

        double frontMin = Math.min(fldir, frdir);
        double backMin = Math.min(bldir, brdir);

        String direction = "";
        double leftMax = Math.max(flpower, blpower);
        double rightMax = Math.max(frpower, brpower);
        packet.clearLines();
        if (fldir != 0 || frdir != 0 || bldir != 0 || brdir != 0) {
            if (fldir == 1 && bldir == 1 && frdir == 1 && brdir == 1)
                direction = "Moving Forward";
            if (fldir == -1 && bldir == -1 && frdir == -1 && brdir == -1)
                direction = "Moving Backward";
            if (fldir == -1 && bldir == 1 && frdir == 1 && brdir == -1)
                direction = "Strafing Left";
            if (fldir == 1 && bldir == -1 && frdir == -1 && brdir == 1)
                direction = "Strafing Right";
            if (fldir == -1 && bldir == -1 && frdir == 1 && brdir == 1)
                direction = "Turning Counterclockwise";
            if (fldir == 1 && bldir == 1 && frdir == -1 && brdir == -1)
                direction = "Turning Clockwise";
            if (frontMin == 0 && backMin == 0)
                direction = "Moving Diagonally";
            if ((frontMin == 0 && backMin != 0) || (backMin == 0 && frontMin != 0))
                direction = "Moving Strangely";
            telemetry.addLine(direction + " at " + Math.max(leftMax, rightMax) + "% Speed");
            packet.addLine(direction + " at " + Math.max(leftMax, rightMax) + "% Speed");
        } else {

            telemetry.addLine("Stopped");
            packet.addLine("Stopped");

        }

    }

    /**Starts a camera stream pushed to the FTC Dashboard.
     * @param source The camera source
     * @param maxFps The maximum framerate allowed to be sent.*/
    public void startCameraStream(CameraStreamSource source, double maxFps) {

        dashboard.startCameraStream(source, maxFps);

    }

    /**Updates telemetry to tell what direction the drivebase is moving for two motor drivebases.
     * @param leftPower the power for the left motor
     * @param rightPower the power for the right motor*/
    public void motorTelemetryUpdate(double leftPower, double rightPower) {

        leftDir = Math.signum(leftPower);
        rightDir = Math.signum(rightPower);

        double frontMin = Math.min(fldir, frdir);
        double backMin = Math.min(bldir, brdir);

        String direction = "";
        packet.clearLines();
        if (leftDir != 0 || rightDir != 0) {
            if (leftDir == 1 && rightDir == 1)
                direction = "Moving Forward";
            if (leftDir == -1 && rightDir == -1)
                direction = "Moving Backward";
            if (leftDir == -1 && rightDir == 1)
                direction = "Turning Counterclockwise";
            if (leftDir == 1 && rightDir == -1)
                direction = "Turning Clockwise";
            telemetry.addLine(direction + " at " + Math.max(leftPower, rightPower) + "% Speed");
            packet.addLine(direction + " at " + Math.max(leftPower, rightPower) + "% Speed");
        } else {

            telemetry.addLine("Stopped");
            packet.addLine("Stopped");

        }

    }

    /**Updates the telemetry and dashboard.*/
    public void update() {

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);

    }

}
