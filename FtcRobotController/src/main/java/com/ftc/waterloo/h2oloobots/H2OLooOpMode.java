package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class H2OLooOpMode extends LinearOpMode {

    public H2OLooOpMode() {}
    public TelemetryControl telemetryControl;

    public DriveTrain driveTrain;
    public AttachmentControl attachmentControl;
    public OdometryControl odometryControl;

    public void runOpMode() {

        telemetryControl = new TelemetryControl(telemetry);
        driveTrain = new DriveTrain(hardwareMap, telemetryControl);
        odometryControl = new OdometryControl(hardwareMap, telemetryControl);
        attachmentControl = new AttachmentControl(hardwareMap, telemetryControl);

        this.opModeInit();

        waitForStart();

        this.opModePeriodic();

    }

    abstract public void opModeInit();

    abstract public void opModePeriodic();

}
