package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class H2OLooMaster {

    Telemetry telemetry;
    HardwareMap hardwareMap;

    TelemetryControl telemetryControl;
    DriveTrain driveTrain;
    OdometryControl odometryControl;
    AttachmentControl attachmentControl;

    public H2OLooMaster(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        telemetryControl = new TelemetryControl(telemetry);
        driveTrain = new DriveTrain(hardwareMap, telemetryControl);
        odometryControl = new OdometryControl(hardwareMap, telemetryControl);
        attachmentControl = new AttachmentControl(hardwareMap, telemetryControl);

    }

    public Object[] returnGlobalVariables() {

        return new Object[]{telemetryControl, driveTrain, odometryControl, attachmentControl};

    }

}
