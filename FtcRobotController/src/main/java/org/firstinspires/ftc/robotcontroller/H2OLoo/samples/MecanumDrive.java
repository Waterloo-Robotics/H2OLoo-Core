package org.firstinspires.ftc.robotcontroller.H2OLoo.samples;

import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.H2OLooOpMode;
import com.ftc.waterloo.h2oloobots.TelemetryControl;

public class MecanumDrive extends H2OLooOpMode {

    @Override
    public void opModeInit() {

        driveTrain.setDriveTrainType(DriveTrain.DriveTrainType.MECANUM);

    }

    @Override
    public void opModePeriodic() {

        driveTrain.teleOpDrive(gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

    }

}
