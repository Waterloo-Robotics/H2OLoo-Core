package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.DriveTrain;
import com.ftc.waterloo.h2oloobots.H2OLooOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Sebastian extends H2OLooOpMode {

    double turn = 0;

    public void opModeInit() {

        driveTrain.setDriveTrainType(DriveTrain.DriveTrainType.MECANUM);
    }

    public void opModePeriodic() {

        turn = gamepad1.right_trigger - gamepad1.left_trigger;
        driveTrain.teleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, turn);

    }

}
