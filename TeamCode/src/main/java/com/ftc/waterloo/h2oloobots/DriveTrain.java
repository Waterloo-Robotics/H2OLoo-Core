package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain {

    public enum DriveTrainType {

        TWO_WHEEL_DRIVE,
        FOUR_WHEEL_TANK,
        MECANUM

    }

    public enum DriveDirection {

        FORWARD,
        BACKWARD,
        STRAFE_LEFT,
        STRAFE_RIGHT,
        TURN_CLOCKWISE,
        TURN_COUNTERCLOCKWISE,
        DIAGONAL_FORWARD_RIGHT,
        DIAGONAL_FORWARD_LEFT,
        DIAGONAL_BACKWARD_RIGHT,
        DIAGONAL_BACKWARD_LEFT,

    }

    DriveTrainType driveTrainType;

    // defines drive motors for a 4 wheel drive
    DcMotor fl, fr, bl, br;

    // defines drive motors for a two wheel drive
    DcMotor left, right;

    /* counts per revolution of the drive motors. None of this is necessary if you aren't using
     * the encoder drive in this file. */
    double countsPerRevolution = 553;

    // diameter of your drive wheels
    double wheelDiameter = 96 / 25.4; // inches
    double wheelCircumference = wheelDiameter * Math.PI;

    /*
     * Track Width is the distance between the two sets of wheels (defined by the line of x below).
     *
     * TODO this will need to be updated for turns to work. it might not be the actual value
     *  that is physically measured.
     *
     *          FRONT
     *  O--------------------O
     *  |                    |
     *  |                    |
     *  |                    |
     *  |xxxxxxxxxxxxxxxxxxxx|
     *  |                    |
     *  |                    |
     *  |                    |
     *  O--------------------O
     */
    double trackWidth = 1.0;
    // how many inches the wheels travel in a full rotation of the robot
    double fullRotation = trackWidth * Math.PI;

    // calculates how many encoder counts are in an inch and in a degree
    double COUNTS_PER_INCH = countsPerRevolution / wheelCircumference;
    double COUNTS_PER_DEGREE = fullRotation / wheelCircumference * countsPerRevolution / 360.0;

    // defines local HardwareMap and TelemetryControl variables.
    HardwareMap hardwareMap;
    TelemetryControl telemetryControl;

    /**Initialises the drivetrain variable.
     * @param hardwareMap the local HardwareMap variable from in the runOpMode() void.
     * @param telemetryControl the TelemetryControl variable initialized in the runOpMode() void.
     * @param driveTrainType the drivetrain type: can be two wheel, 4 wheel tank, or mecanum.*/
    public DriveTrain(HardwareMap hardwareMap,
                      TelemetryControl telemetryControl,
                      DriveTrainType driveTrainType
    ) {

        this.hardwareMap = hardwareMap;
        this.telemetryControl = telemetryControl;
        this.driveTrainType = driveTrainType;

        switch (driveTrainType) {

            case TWO_WHEEL_DRIVE:
                this.TwoWheelInit();
                break;

            case FOUR_WHEEL_TANK:
            case MECANUM:
                this.FourMotorInit();
                break;

        }

    }

    /**Initialises the drivetrain variable.
     * @param hardwareMap the local HardwareMap variable from in the runOpMode() void.
     * @param telemetryControl the TelemetryControl variable initialized in the runOpMode() void.
     * @param driveTrainType the drivetrain type: can be two wheel, 4 wheel tank, or mecanum.
     * @param zeroPowerBehavior the zero power behavior to be set to the drive motors.*/
    public DriveTrain(
            HardwareMap hardwareMap,
            TelemetryControl telemetryControl,
            DriveTrainType driveTrainType,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior
    ) {

        this.hardwareMap = hardwareMap;
        this.telemetryControl = telemetryControl;
        this.driveTrainType = driveTrainType;

        switch (driveTrainType) {

            case TWO_WHEEL_DRIVE:
                this.TwoWheelInit(zeroPowerBehavior);
                break;

            case FOUR_WHEEL_TANK:
            case MECANUM:
                this.FourMotorInit(zeroPowerBehavior);
                break;

        }

    }

    /**Initialization for two motor drivebase (one left, one right).*/
    void TwoWheelInit() {

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**Initialization for two motor drivebase (one left, one right).
     * @param zeroPowerBehavior the zero power behavior to set to the drive motors.*/
    void TwoWheelInit(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(zeroPowerBehavior);
        right.setZeroPowerBehavior(zeroPowerBehavior);

    }

    /**Four motor initialization command for any four motor drive base with two motors on either
     * side. Could be mecanum or tank.*/
    void FourMotorInit() {

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /**Four motor initialization command for any four motor drive base with two motors on either
     * side. Could be mecanum or tank.
     * @param zeroPowerBehavior the zero power behavior to set to the motors.*/
    void FourMotorInit(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setZeroPowerBehavior(zeroPowerBehavior);
        fr.setZeroPowerBehavior(zeroPowerBehavior);
        bl.setZeroPowerBehavior(zeroPowerBehavior);
        br.setZeroPowerBehavior(zeroPowerBehavior);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /**TeleOp code for a 4 wheel tank or two wheel drive.
     * @param FBInput forward and back input, range -1 to 1
     * @param pivotInput pivot input, range -1 to 1*/
    public void teleOpDrive(double FBInput, double pivotInput) {

        switch (driveTrainType) {

            case TWO_WHEEL_DRIVE:
                this.TwoWheelDriveTeleOp(FBInput, pivotInput);
                break;

            case FOUR_WHEEL_TANK:
                this.FWDTeleOp(FBInput, pivotInput);
                break;

            case MECANUM:
                RobotLog.setGlobalErrorMsg("Passing in a LRInput is necessary. Please use the " +
                        "version of this function that uses 'teleOpDrive(FBInput, LRInput," +
                        " pivotInput)'.");

        }

    }

    /**TeleOp code for a mecanum drive.
     * @param FBInput forward and back input, range -1 to 1
     * @param LRInput left and right strafing input, range -1 to 1
     * @param pivotInput pivot input, range -1 to 1*/
    public void teleOpDrive(double FBInput, double LRInput, double pivotInput) {

        switch (driveTrainType) {

            case TWO_WHEEL_DRIVE:
            case FOUR_WHEEL_TANK:
                RobotLog.setGlobalErrorMsg("Passing in a LRInput is unacceptable. Please use the " +
                        "version of this function that uses 'teleOpDrive(FBInput," +
                        " pivotInput)'.");
                break;

            case MECANUM:
                this.MecanumTeleOp(FBInput, LRInput, pivotInput);
                break;

        }

    }

    /**Simple two wheel drive TeleOp.
     * @param FBInput input used for forward and back movements.
     * @param PivotInput input used for turning.*/
    public void TwoWheelDriveTeleOp(double FBInput, double PivotInput) {

        right.setPower(-FBInput - PivotInput);
        left.setPower(-FBInput + PivotInput);

    }

    void FWDTeleOp(double FBInput, double PivotInput) {

        fr.setPower(-FBInput - PivotInput);
        fl.setPower(-FBInput + PivotInput);
        br.setPower(-FBInput - PivotInput);
        bl.setPower(-FBInput + PivotInput);

    }

    void MecanumTeleOp(double FBInput, double LRInput, double PivotInput) {

        fr.setPower((-FBInput - LRInput - (PivotInput)));
        br.setPower((-FBInput + LRInput - (PivotInput)));
        fl.setPower((-FBInput + LRInput + (PivotInput)));
        bl.setPower((-FBInput - LRInput + (PivotInput)));

        telemetryControl.motorTelemetryUpdate(fl.getPower(), fr.getPower(), bl.getPower(), br.getPower());

    }

    void timeAutoFourWheelDrive(double FRPower, double FLPower, double BRPower, double BLPower, double SECONDS) {

        ElapsedTime time = new ElapsedTime();

        time.reset();

        while (time.seconds() < SECONDS) {

            fr.setPower(FRPower);
            fl.setPower(FLPower);
            br.setPower(BRPower);
            bl.setPower(BLPower);

        }

        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);

    }

    void timeAutoFourWheelDrive(DriveDirection driveDirection, double power, double SECONDS) {

        //TODO fix this
        double frPower = power;
        double flPower = power;
        double brPower = power;
        double blPower = power;

        switch (driveDirection) {

            case FORWARD:
                frPower *= 1;
                flPower *= 1;
                brPower *= 1;
                blPower *= 1;
            break;

            case BACKWARD:
                frPower *= -1;
                flPower *= -1;
                brPower *= -1;
                blPower *= -1;
            break;

            case STRAFE_LEFT:
                frPower *= 1;
                flPower *= -1;
                brPower *= -1;
                blPower *= 1;
            break;

            case STRAFE_RIGHT:
                frPower *= -1;
                flPower *= 1;
                brPower *= 1;
                blPower *= -1;
            break;

            case TURN_CLOCKWISE:
                frPower *= 1;
                flPower *= -1;
                brPower *= 1;
                blPower *= -1;
            break;

        }

        this.timeAutoFourWheelDrive(frPower, flPower, brPower, blPower, SECONDS);

    }

    void EncoderAutoMecanumDrive(double INCHES_FB, double INCHES_LR, double DEGREES_TURN, double SPEED, int time) {

        ElapsedTime timer = new ElapsedTime();

        if (DEGREES_TURN > 180) {

            DEGREES_TURN -= 360;

        }

        int frTargetPosition = fr.getCurrentPosition() + (int) (this.COUNTS_PER_INCH * INCHES_FB) - (int) (this.COUNTS_PER_INCH * INCHES_LR) - (int) (this.COUNTS_PER_DEGREE * DEGREES_TURN);
        int brTargetPosition = br.getCurrentPosition() + (int) (this.COUNTS_PER_INCH * INCHES_FB) + (int) (this.COUNTS_PER_INCH * INCHES_LR) - (int) (this.COUNTS_PER_DEGREE * DEGREES_TURN);
        int flTargetPosition = fl.getCurrentPosition() - (int) (COUNTS_PER_INCH * INCHES_FB) - (int) (this.COUNTS_PER_INCH * INCHES_LR) - (int) (this.COUNTS_PER_DEGREE * DEGREES_TURN);
        int blTargetPosition = bl.getCurrentPosition() - (int) (this.COUNTS_PER_INCH * INCHES_FB) + (int) (this.COUNTS_PER_INCH * INCHES_LR) - (int) (this.COUNTS_PER_DEGREE * DEGREES_TURN);

        fr.setTargetPosition(frTargetPosition);
        br.setTargetPosition(brTargetPosition);
        fl.setTargetPosition(flTargetPosition);
        bl.setTargetPosition(blTargetPosition);

        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        timer.reset();

        fr.setPower(SPEED);
        br.setPower(SPEED);
        fl.setPower(SPEED);
        bl.setPower(SPEED);

        while ((fr.isBusy() || br.isBusy() || fl.isBusy() || bl.isBusy()) && timer.seconds() <= time) {

        }

        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.reset();
        while (waitTimer.seconds() <= 0.125) {

        }

    }

}
