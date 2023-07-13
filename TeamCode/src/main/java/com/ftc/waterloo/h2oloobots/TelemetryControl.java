package com.ftc.waterloo.h2oloobots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryControl {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    Telemetry telemetry;

    double fldir = 0;
    double frdir = 0;
    double bldir = 0;
    double brdir = 0;

    public TelemetryControl(Telemetry telemetry) {

        this.telemetry = telemetry;
        packet.addLine("Robot Initialised");
        telemetry.addLine("Robot Initialised");

    }

    public void addData(String caption, Object value) {

        telemetry.addData(caption, value);
        packet.put(caption, value);

    }

    public void motorTelemetryUpdate(double flpower, double frpower, double blpower, double brpower) {

        fldir = getDirection(flpower);
        frdir = getDirection(frpower);
        bldir = getDirection(blpower);
        brdir = getDirection(brpower);

        double frontMin = Math.min(fldir, frdir);
        double backMin = Math.min(bldir, brdir);

        String direction = "";
        double leftMax = Math.max(flpower, blpower);
        double rightMax = Math.max(frpower, brpower);
        packet.clearLines();
        if (fldir != 0 && frdir != 0 && bldir != 0 && brdir != 0) {
            if (fldir == -1 && bldir == -1 && frdir == 1 && brdir == 1)
                direction = "Moving Forward";
            if (fldir == 1 && bldir == 1 && frdir == -1 && brdir == -1)
                direction = "Moving Backward";
            if (fldir == 1 && bldir == -1 && frdir == 1 && brdir == -1)
                direction = "Strafing Left";
            if (fldir == -1 && bldir == 1 && frdir == -1 && brdir == 1)
                direction = "Strafing Right";
            if (fldir == 1 && bldir == 1 && frdir == 1 && brdir == 1)
                direction = "Turning Left";
            if (fldir == -1 && bldir == -1 && frdir == -1 && brdir == -1)
                direction = "Turning Right";
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

    public void addLine(String line) {

        telemetry.addLine(line);
        packet.clearLines();
        packet.addLine(line);

    }

    public void update() {

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);

    }

    public double getDirection(double motorPower) {

        if (motorPower != 0) {

            return (motorPower) / Math.abs(motorPower);

        } else {

            return 0;

        }

    }

}
