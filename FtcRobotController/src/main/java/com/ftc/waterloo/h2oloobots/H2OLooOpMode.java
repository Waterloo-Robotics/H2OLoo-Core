package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class H2OLooOpMode extends LinearOpMode {

    public H2OLooOpMode() {}
    public TelemetryControl telemetryControl = new TelemetryControl(telemetry);

    public DriveTrain driveTrain = new DriveTrain(hardwareMap, telemetryControl, null);
    public AttachmentControl attachmentControl = new AttachmentControl(hardwareMap, telemetryControl);

    public void runOpMode() {

        this.opModeInit();

        waitForStart();

        this.opModePeriodic();

    }

    abstract public void opModeInit();

    abstract public void opModePeriodic();

}
