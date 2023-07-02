package com.ftc.waterloo.h2oloobots;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class OdometryControl extends LinearOpMode {

    public DcMotorEx fl, fr, bl, br;

    public Encoder leftEncoder, rightEncoder, horizEncoder;

    public static double P_gen_forwardback = 0.03;
    public static double P_gen_strafe = 0.05;
    public static double P_gen_turn = 0.03;
    public static double P_differential = 0.1;


    public static double MAX_POWER = 0.85;
    public static double MIN_POWER_STRAIGHT = 0.3;
    public static double MIN_POWER_TURNSTRAFE = 0.55;

    public static double DISTANCE = 53;

    public static double deadWheelDiameter = 35.0; // measured in mm

    HardwareMap hardwareMapLocal;
    TelemetryControl telemetryControlLocal;

    ElapsedTime time = new ElapsedTime();

    public OdometryControl(HardwareMap hardwareMap, TelemetryControl telemetryControl) {

        fl = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        fr = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        bl = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        br = (DcMotorEx) hardwareMap.dcMotor.get("br");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // use any of the following if motors need reversed
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
//        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
//        br.setDirection(DcMotorSimple.Direction.REVERSE);

        leftEncoder = new Encoder(
                hardwareMap,
                "fl",
                8192,
                deadWheelDiameter,
                Encoder.MeasurementUnit.MM
        );
        rightEncoder = new Encoder(
                hardwareMap,
                "fr",
                8192,
                deadWheelDiameter,
                Encoder.MeasurementUnit.MM
        );
        horizEncoder = new Encoder(
                hardwareMap,
                "br",
                8192,
                deadWheelDiameter,
                Encoder.MeasurementUnit.MM
        );

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hardwareMapLocal = hardwareMap;
        telemetryControlLocal = telemetryControl;

    }

    public void runOpMode() {}

    /** This just resets encoder counts, makes life easy for our side of things. */
    public void zeroEncoders() {

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    /**Drive forward a set amount of inches, with a set timeout just in case something goes terribly wrong.*/
    public void forward(double INCHES, double TIME) {

        double rightOffset = 0, leftOffset = 0, genPower = 0, rightPower = 0, leftPower = 0;

        double distanceTravelled = 0;
        double error = 0;
        double differentialError = 0;

        double rightTravelled, leftTravelled;

        boolean destinationReached = false;

        double seconds = 0;
        zeroEncoders(); // resets encoders

        boolean waited = false;

        time.reset();
        // while loop that only ends once distance is reached or it times out
        while (!destinationReached && time.seconds() < 10 + TIME && !isStopRequested()) {

            rightTravelled = rightEncoder.getDistance(Encoder.MeasurementUnit.IN);
            leftTravelled = -leftEncoder.getDistance(Encoder.MeasurementUnit.IN);

            distanceTravelled = ((leftTravelled + rightTravelled) / 2.0);

            error = distanceTravelled - INCHES;

            differentialError = rightTravelled - leftTravelled;

            if (DISTANCE > 10) { // speeds up 5x if distance is less than 10 inches

                genPower = (genPower + (error * P_gen_forwardback)) / 2.0;

            } else {

                genPower = (genPower + (error * P_gen_forwardback * 5)) / 2.0;

            }

            if (differentialError > 0) { //Turning left, right faster
                rightOffset = (rightOffset + ((rightTravelled - leftTravelled) * P_differential)) / 2.0;
                leftOffset = 0;
            } else {
                rightOffset = 0;
                leftOffset = (leftOffset + ((leftTravelled - rightTravelled) * P_differential)) / 2.0;
            }

            telemetryControlLocal.addData("distanceTravelled", String.valueOf(distanceTravelled));
            telemetryControlLocal.addData("Right Offset", String.valueOf(rightOffset));
            telemetryControlLocal.addData("Left Offset", String.valueOf(leftOffset));
            telemetryControlLocal.addData("Seconds", String.valueOf(seconds));

            telemetryControlLocal.addData("Diff Error", String.valueOf(differentialError));
            telemetryControlLocal.addData("GenPower", String.valueOf(genPower));
            telemetryControlLocal.addData("fl", fl.getPower());
            telemetryControlLocal.addData("fr", fr.getPower());
            telemetryControlLocal.addData("bl", bl.getPower());
            telemetryControlLocal.addData("br", br.getPower());
            telemetryControlLocal.update();


            //Calc side powers
            rightPower = genPower + rightOffset * genPower;
            leftPower = genPower + leftOffset * genPower;

            //Cap +/- at MAX_POWER
            if (rightPower > MAX_POWER) {
                rightPower = MAX_POWER;
            } else if (rightPower < -MAX_POWER) {
                rightPower = -MAX_POWER;
            }

            if (leftPower > MAX_POWER) {
                leftPower = MAX_POWER;
            } else if (leftPower < -MAX_POWER) {
                leftPower = -MAX_POWER;
            }

            //Cap +/- at MIN_POWER
            if (rightPower < MIN_POWER_STRAIGHT && rightPower > 0) {
                rightPower = MIN_POWER_STRAIGHT;
            } else if (rightPower > -MIN_POWER_STRAIGHT && rightPower < 0) {
                rightPower = -MIN_POWER_STRAIGHT;
            }

            if (leftPower < MIN_POWER_STRAIGHT && leftPower > 0) {
                leftPower = MIN_POWER_STRAIGHT;
            } else if (leftPower > -MIN_POWER_STRAIGHT && leftPower < 0) {
                leftPower = -MIN_POWER_STRAIGHT;
            }

            if (!waited) {
                time.reset();

                while (time.seconds() < TIME);
                waited = true;
            }

            //Set motor powers
            fr.setPower(rightPower);
            br.setPower(rightPower);
            fl.setPower(leftPower);
            bl.setPower(leftPower);

            if (distanceTravelled > INCHES) {
                destinationReached = true;
            }

        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }

    public void back(double INCHES, double TIME) {

        double rightOffset = 0, leftOffset = 0, genPower = 0, rightPower = 0, leftPower = 0;

        double distanceTravelled = 0;
        double error = 0;
        double rightTravelled, leftTravelled;
        double differentialError = 0;

        boolean destinationReached = false;

        double seconds = 0;
        zeroEncoders();

        boolean waited = false;

        time.reset();
        while (!destinationReached && time.seconds() < 10 + TIME && !isStopRequested()) {

            rightTravelled = -rightEncoder.getDistance(Encoder.MeasurementUnit.IN);
            leftTravelled = leftEncoder.getDistance(Encoder.MeasurementUnit.IN);

            distanceTravelled = ((leftTravelled + rightTravelled) / 2.0);

            error = distanceTravelled - INCHES;

            differentialError = rightTravelled - leftTravelled;

            if (DISTANCE > 10) {

                genPower = (genPower + (error * P_gen_forwardback)) / 2.0;

            } else {

                genPower = (genPower + (error * P_gen_forwardback * 5)) / 2.0;

            }

            if (differentialError > 0) { //Turning left, right faster
                rightOffset = (rightOffset + ( (rightTravelled - leftTravelled) * P_differential) )/ 2.0;
                leftOffset = 0;
            } else {
                rightOffset = 0;
                leftOffset = (leftOffset + ( (leftTravelled - rightTravelled) * P_differential) ) / 2.0;
            }

            telemetryControlLocal.addData("distanceTravelled", String.valueOf(distanceTravelled));
            telemetryControlLocal.addData("Right Offset", String.valueOf(rightOffset));
            telemetryControlLocal.addData("Left Offset", String.valueOf(leftOffset));
            telemetryControlLocal.addData("Seconds", String.valueOf(seconds));

            telemetryControlLocal.addData("Diff Error", String.valueOf(differentialError));
            telemetryControlLocal.addData("GenPower", String.valueOf(genPower));
            telemetryControlLocal.addData("fl", fl.getPower());
            telemetryControlLocal.addData("fr", fr.getPower());
            telemetryControlLocal.addData("bl", bl.getPower());
            telemetryControlLocal.addData("br", br.getPower());
            telemetryControlLocal.update();


            //Calc side powers
            rightPower = genPower + rightOffset * genPower;
            leftPower = genPower + leftOffset * genPower;

            //Cap +/- at MAX_POWER
            if (rightPower > MAX_POWER) {
                rightPower = MAX_POWER;
            } else if (rightPower < -MAX_POWER) {
                rightPower = -MAX_POWER;
            }

            if (leftPower > MAX_POWER) {
                leftPower = MAX_POWER;
            } else if (leftPower < -MAX_POWER) {
                leftPower = -MAX_POWER;
            }

            //Cap +/- at MIN_POWER
            if (rightPower < MIN_POWER_STRAIGHT && rightPower > 0) {
                rightPower = MIN_POWER_STRAIGHT;
            } else if (rightPower > -MIN_POWER_STRAIGHT && rightPower < 0) {
                rightPower = -MIN_POWER_STRAIGHT;
            }

            if (leftPower < MIN_POWER_STRAIGHT && leftPower > 0) {
                leftPower = MIN_POWER_STRAIGHT;
            } else if (leftPower > -MIN_POWER_STRAIGHT && leftPower < 0) {
                leftPower = -MIN_POWER_STRAIGHT;
            }

            if (!waited) {
                time.reset();

                while (time.seconds() < TIME);
                waited = true;
            }

            //Set motor powers
            fr.setPower(-rightPower);
            br.setPower(-rightPower);
            fl.setPower(-leftPower);
            bl.setPower(-leftPower);

            if (distanceTravelled > INCHES) {
                destinationReached = true;
            }

        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }

    public void back(double INCHES, double TIME, double SPEED) {

        double rightOffset = 0, leftOffset = 0, genPower = 0, rightPower = 0, leftPower = 0;

        double distanceTravelled = 0;
        double error = 0;
        double rightTravelled, leftTravelled;
        double differentialError = 0;

        boolean destinationReached = false;

        double seconds = 0;
        zeroEncoders();

        boolean waited = false;

        time.reset();
        while (!destinationReached && time.seconds() < 10 + TIME && !isStopRequested()) {

            rightTravelled = -rightEncoder.getDistance(Encoder.MeasurementUnit.IN);
            leftTravelled = leftEncoder.getDistance(Encoder.MeasurementUnit.IN);

            distanceTravelled = ((leftTravelled + rightTravelled) / 2.0);

            error = distanceTravelled - INCHES;

            differentialError = rightTravelled - leftTravelled;

            if (DISTANCE > 10) {

                genPower = (genPower + (error * P_gen_forwardback * 15)) / 2.0;

            } else {

                genPower = (genPower + (error * P_gen_forwardback * 15)) / 2.0;

            }

            if (differentialError > 0) { //Turning left, right faster
                rightOffset = (rightOffset + ( (rightTravelled - leftTravelled) * P_differential) )/ 2.0;
                leftOffset = 0;
            } else {
                rightOffset = 0;
                leftOffset = (leftOffset + ( (leftTravelled - rightTravelled) * P_differential) ) / 2.0;
            }

            telemetryControlLocal.addData("distanceTravelled", String.valueOf(distanceTravelled));
            telemetryControlLocal.addData("Right Offset", String.valueOf(rightOffset));
            telemetryControlLocal.addData("Left Offset", String.valueOf(leftOffset));
            telemetryControlLocal.addData("Seconds", String.valueOf(seconds));

            telemetryControlLocal.addData("Diff Error", String.valueOf(differentialError));
            telemetryControlLocal.addData("GenPower", String.valueOf(genPower));
            telemetryControlLocal.addData("fl", fl.getPower());
            telemetryControlLocal.addData("fr", fr.getPower());
            telemetryControlLocal.addData("bl", bl.getPower());
            telemetryControlLocal.addData("br", br.getPower());
            telemetryControlLocal.update();


            //Calc side powers
            rightPower = genPower + rightOffset * genPower;
            leftPower = genPower + leftOffset * genPower;

            //Cap +/- at MAX_POWER
            if (rightPower > MAX_POWER) {
                rightPower = MAX_POWER;
            } else if (rightPower < -MAX_POWER) {
                rightPower = -MAX_POWER;
            }

            if (leftPower > MAX_POWER) {
                leftPower = MAX_POWER;
            } else if (leftPower < -MAX_POWER) {
                leftPower = -MAX_POWER;
            }

            //Cap +/- at MIN_POWER
            if (rightPower < MIN_POWER_STRAIGHT && rightPower > 0) {
                rightPower = MIN_POWER_STRAIGHT;
            } else if (rightPower > -MIN_POWER_STRAIGHT && rightPower < 0) {
                rightPower = -MIN_POWER_STRAIGHT;
            }

            if (leftPower < MIN_POWER_STRAIGHT && leftPower > 0) {
                leftPower = MIN_POWER_STRAIGHT;
            } else if (leftPower > -MIN_POWER_STRAIGHT && leftPower < 0) {
                leftPower = -MIN_POWER_STRAIGHT;
            }

            if (!waited) {
                time.reset();

                while (time.seconds() < TIME);
                waited = true;
            }

            //Set motor powers
            fr.setPower(-rightPower);
            br.setPower(-rightPower);
            fl.setPower(-leftPower);
            bl.setPower(-leftPower);

            if (distanceTravelled > INCHES) {
                destinationReached = true;
            }

        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }

    public void strafeLeft(double INCHES, double TIME) {

        double rightOffset = 0, leftOffset = 0, genPower = 0, rightPower = 0, leftPower = 0;

        double distanceTravelled = 0;
        double distanceTravelledForward = 0;
        double error = 0;
        double headingError = 0;

        double rightTravelled, leftTravelled, horizTravelled;

        boolean destinationReached = false;

        double seconds = 0;
        zeroEncoders();

        boolean waited = false;

        time.reset();
        while (!destinationReached && time.seconds() < 6 + TIME && !isStopRequested()) {

            rightTravelled = rightEncoder.getDistance(Encoder.MeasurementUnit.IN);
            leftTravelled = -leftEncoder.getDistance(Encoder.MeasurementUnit.IN);
            horizTravelled = -horizEncoder.getDistance(Encoder.MeasurementUnit.IN);

            distanceTravelledForward = ((leftTravelled + rightTravelled) / 2.0);
            distanceTravelled = horizTravelled;

            error = distanceTravelled - INCHES;

            headingError = rightTravelled - leftTravelled;

            genPower = (genPower + (error * P_gen_strafe)) / 2.0;

            telemetryControlLocal.addData("distanceTravelledForward", String.valueOf(distanceTravelledForward));
            telemetryControlLocal.addData("Right Offset", String.valueOf(rightOffset));
            telemetryControlLocal.addData("Left Offset", String.valueOf(leftOffset));
            telemetryControlLocal.addData("Seconds", String.valueOf(seconds));

            telemetryControlLocal.addData("Diff Error", String.valueOf(headingError));
            telemetryControlLocal.addData("GenPower", String.valueOf(genPower));
            telemetryControlLocal.addData("fl", fl.getPower());
            telemetryControlLocal.addData("fr", fr.getPower());
            telemetryControlLocal.addData("bl", bl.getPower());
            telemetryControlLocal.addData("br", br.getPower());
            telemetryControlLocal.update();


            //Calc side powers
            rightPower = genPower + rightOffset;
            leftPower = genPower + leftOffset;

            //Cap +/- at MAX_POWER
            if (rightPower > MAX_POWER) {
                rightPower = MAX_POWER;
            } else if (rightPower < -MAX_POWER) {
                rightPower = -MAX_POWER;
            }

            if (leftPower > MAX_POWER) {
                leftPower = MAX_POWER;
            } else if (leftPower < -MAX_POWER) {
                leftPower = -MAX_POWER;
            }

            //Cap +/- at MIN_POWER
            if (rightPower < MIN_POWER_TURNSTRAFE && rightPower > 0) {
                rightPower = MIN_POWER_TURNSTRAFE;
            } else if (rightPower > -MIN_POWER_TURNSTRAFE && rightPower < 0) {
                rightPower = -MIN_POWER_TURNSTRAFE;
            }

            if (leftPower < MIN_POWER_TURNSTRAFE && leftPower > 0) {
                leftPower = MIN_POWER_TURNSTRAFE;
            } else if (leftPower > -MIN_POWER_TURNSTRAFE && leftPower < 0) {
                leftPower = -MIN_POWER_TURNSTRAFE;
            }

            if (!waited) {
                time.reset();

                while (time.seconds() < TIME);
                waited = true;
            }

            //Set motor powers
            fr.setPower(rightPower);
            br.setPower(-rightPower);
            fl.setPower(-leftPower);
            bl.setPower(leftPower);

            if (distanceTravelled > INCHES) {
                destinationReached = true;
            }

        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }

    public void strafeRight(double INCHES, double TIME) {

        double rightOffset = 0, leftOffset = 0, genPower = 0, rightPower = 0, leftPower = 0;

        double distanceTravelled = 0;
        double distanceTravelledForward = 0;
        double error = 0;
        double headingError = 0;

        double rightTravelled, leftTravelled, horizTravelled;

        boolean destinationReached = false;

        double seconds = 0;
        zeroEncoders();

        boolean waited = false;

        time.reset();
        while (!destinationReached && time.seconds() < 6 + TIME && !isStopRequested()) {

            rightTravelled = rightEncoder.getDistance(Encoder.MeasurementUnit.IN);
            leftTravelled = -leftEncoder.getDistance(Encoder.MeasurementUnit.IN);
            horizTravelled = horizEncoder.getDistance(Encoder.MeasurementUnit.IN);

            distanceTravelledForward = ((leftTravelled + rightTravelled) / 2.0);
            distanceTravelled = horizTravelled;

            error = distanceTravelled - INCHES;

            headingError = rightTravelled - leftTravelled;

            genPower = (genPower + (error * P_gen_strafe)) / 2.0;

            telemetryControlLocal.addData("distanceTravelledForward", String.valueOf(distanceTravelledForward));
            telemetryControlLocal.addData("Right Offset", String.valueOf(rightOffset));
            telemetryControlLocal.addData("Left Offset", String.valueOf(leftOffset));
            telemetryControlLocal.addData("Seconds", String.valueOf(seconds));

            telemetryControlLocal.addData("Diff Error", String.valueOf(headingError));
            telemetryControlLocal.addData("GenPower", String.valueOf(genPower));
            telemetryControlLocal.addData("fl", fl.getPower());
            telemetryControlLocal.addData("fr", fr.getPower());
            telemetryControlLocal.addData("bl", bl.getPower());
            telemetryControlLocal.addData("br", br.getPower());
            telemetryControlLocal.update();


            //Calc side powers
            rightPower = genPower + rightOffset;
            leftPower = genPower + leftOffset;

            //Cap +/- at MAX_POWER
            if (rightPower > MAX_POWER) {
                rightPower = MAX_POWER;
            } else if (rightPower < -MAX_POWER) {
                rightPower = -MAX_POWER;
            }

            if (leftPower > MAX_POWER) {
                leftPower = MAX_POWER;
            } else if (leftPower < -MAX_POWER) {
                leftPower = -MAX_POWER;
            }

            //Cap +/- at MIN_POWER
            if (rightPower < MIN_POWER_TURNSTRAFE && rightPower > 0) {
                rightPower = MIN_POWER_TURNSTRAFE;
            } else if (rightPower > -MIN_POWER_TURNSTRAFE && rightPower < 0) {
                rightPower = -MIN_POWER_TURNSTRAFE;
            }

            if (leftPower < MIN_POWER_TURNSTRAFE && leftPower > 0) {
                leftPower = MIN_POWER_TURNSTRAFE;
            } else if (leftPower > -MIN_POWER_TURNSTRAFE && leftPower < 0) {
                leftPower = -MIN_POWER_TURNSTRAFE;
            }

            if (!waited) {
                time.reset();

                while (time.seconds() < TIME);
                waited = true;
            }

            //Set motor powers
            fr.setPower(-rightPower);
            br.setPower(rightPower);
            fl.setPower(leftPower);
            bl.setPower(-leftPower);

            if (distanceTravelled > INCHES) {
                destinationReached = true;
            }

        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }

    public void turnRight(double INCHES, double TIME) {

        double rightOffset = 0, leftOffset = 0, genPower = 0, rightPower = 0, leftPower = 0;

        double distanceTravelled = 0;
        double error = 0;
        double differentialError = 0;

        double rightTravelled, leftTravelled;

        boolean destinationReached = false;

        double seconds = 0;
        zeroEncoders();

        boolean waited = false;

        time.reset();
        while (!destinationReached && time.seconds() < 10 + TIME && !isStopRequested()) {

            rightTravelled = -rightEncoder.getDistance(Encoder.MeasurementUnit.IN);
            leftTravelled = -leftEncoder.getDistance(Encoder.MeasurementUnit.IN);

            distanceTravelled = ((leftTravelled + rightTravelled) / 2.0);

            error = distanceTravelled - INCHES;

            genPower = (genPower + (error * P_gen_turn)) / 2.0;

            telemetryControlLocal.addData("distanceTravelled", String.valueOf(distanceTravelled));
            telemetryControlLocal.addData("Right Offset", String.valueOf(rightOffset));
            telemetryControlLocal.addData("Left Offset", String.valueOf(leftOffset));
            telemetryControlLocal.addData("Seconds", String.valueOf(seconds));

            telemetryControlLocal.addData("Diff Error", String.valueOf(differentialError));
            telemetryControlLocal.addData("GenPower", String.valueOf(genPower));
            telemetryControlLocal.addData("fl", fl.getPower());
            telemetryControlLocal.addData("fr", fr.getPower());
            telemetryControlLocal.addData("bl", bl.getPower());
            telemetryControlLocal.addData("br", br.getPower());
            telemetryControlLocal.update();


            //Calc side powers
            rightPower = -genPower;
            leftPower = genPower;

            //Cap +/- at MAX_POWER
            if (rightPower > MAX_POWER) {
                rightPower = MAX_POWER;
            } else if (rightPower < -MAX_POWER) {
                rightPower = -MAX_POWER;
            }

            if (leftPower > MAX_POWER) {
                leftPower = MAX_POWER;
            } else if (leftPower < -MAX_POWER) {
                leftPower = -MAX_POWER;
            }

            //Cap +/- at MIN_POWER
            if (rightPower < MIN_POWER_TURNSTRAFE && rightPower > 0) {
                rightPower = MIN_POWER_TURNSTRAFE;
            } else if (rightPower > -MIN_POWER_TURNSTRAFE && rightPower < 0) {
                rightPower = -MIN_POWER_TURNSTRAFE;
            }

            if (leftPower < MIN_POWER_TURNSTRAFE && leftPower > 0) {
                leftPower = MIN_POWER_TURNSTRAFE;
            } else if (leftPower > -MIN_POWER_TURNSTRAFE && leftPower < 0) {
                leftPower = -MIN_POWER_TURNSTRAFE;
            }

            if (!waited) {
                time.reset();

                while (time.seconds() < TIME);
                waited = true;
            }

            //Set motor powers
            fr.setPower(rightPower);
            br.setPower(rightPower);
            fl.setPower(leftPower);
            bl.setPower(leftPower);

            if (distanceTravelled > INCHES) {
                destinationReached = true;
            }

        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }

    public void turnLeft(double INCHES, double TIME) {

        double rightOffset = 0, leftOffset = 0, genPower = 0, rightPower = 0, leftPower = 0;

        double distanceTravelled = 0;
        double error = 0;
        double differentialError = 0;

        double rightTravelled, leftTravelled;

        boolean destinationReached = false;

        double seconds = 0;
        zeroEncoders();

        boolean waited = false;

        while (!destinationReached && time.seconds() < 10 + TIME && !isStopRequested()) {

            rightTravelled = rightEncoder.getDistance(Encoder.MeasurementUnit.IN);
            leftTravelled = leftEncoder.getDistance(Encoder.MeasurementUnit.IN);

            distanceTravelled = ((leftTravelled + rightTravelled) / 2.0);

            error = distanceTravelled - INCHES;

            genPower = (genPower + (error * P_gen_turn)) / 2.0;

            telemetryControlLocal.addData("distanceTravelled", String.valueOf(distanceTravelled));
            telemetryControlLocal.addData("Right Offset", String.valueOf(rightOffset));
            telemetryControlLocal.addData("Left Offset", String.valueOf(leftOffset));
            telemetryControlLocal.addData("Seconds", String.valueOf(seconds));

            telemetryControlLocal.addData("Diff Error", String.valueOf(differentialError));
            telemetryControlLocal.addData("GenPower", String.valueOf(genPower));
            telemetryControlLocal.addData("fl", fl.getPower());
            telemetryControlLocal.addData("fr", fr.getPower());
            telemetryControlLocal.addData("bl", bl.getPower());
            telemetryControlLocal.addData("br", br.getPower());
            telemetryControlLocal.update();


            //Calc side powers
            rightPower = genPower;
            leftPower = -genPower;

            //Cap +/- at MAX_POWER
            if (rightPower > MAX_POWER) {
                rightPower = MAX_POWER;
            } else if (rightPower < -MAX_POWER) {
                rightPower = -MAX_POWER;
            }

            if (leftPower > MAX_POWER) {
                leftPower = MAX_POWER;
            } else if (leftPower < -MAX_POWER) {
                leftPower = -MAX_POWER;
            }

            //Cap +/- at MIN_POWER
            if (rightPower < MIN_POWER_TURNSTRAFE && rightPower > 0) {
                rightPower = MIN_POWER_TURNSTRAFE;
            } else if (rightPower > -MIN_POWER_TURNSTRAFE && rightPower < 0) {
                rightPower = -MIN_POWER_TURNSTRAFE;
            }

            if (leftPower < MIN_POWER_TURNSTRAFE && leftPower > 0) {
                leftPower = MIN_POWER_TURNSTRAFE;
            } else if (leftPower > -MIN_POWER_TURNSTRAFE && leftPower < 0) {
                leftPower = -MIN_POWER_TURNSTRAFE;
            }

            if (!waited) {
                time.reset();

                while (time.seconds() < TIME);
                waited = true;
            }

            //Set motor powers
            fr.setPower(rightPower);
            br.setPower(rightPower);
            fl.setPower(leftPower);
            bl.setPower(leftPower);

            if (distanceTravelled > INCHES) {
                destinationReached = true;
            }

        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }

}
