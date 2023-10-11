package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class H2OLooAuto extends LinearOpMode {

    public H2OLooAuto() {}
    public TelemetryControl telemetryControl;

    public OdometryControl odometryControl;
    public AttachmentControl attachmentControl;

    public void runOpMode() {

        telemetryControl = new TelemetryControl(telemetry);
        odometryControl = new OdometryControl(hardwareMap, telemetryControl);
        attachmentControl = new AttachmentControl(hardwareMap, telemetryControl, gamepad1, gamepad2);

        this.opModeInit();

        waitForStart();

        this.opModePeriodic();

    }

    abstract public void opModeInit();

    abstract public void opModePeriodic();

}
