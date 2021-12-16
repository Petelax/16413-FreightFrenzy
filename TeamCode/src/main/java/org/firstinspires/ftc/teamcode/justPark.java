package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous
public class justPark extends LinearOpMode {

    DcMotor left;
    DcMotor right;
    DcMotor intake;
    DcMotor carousel;
    BNO055IMU imu;
    private static double TICKS_PER_INCH = 651.897;

    @Override
    public void runOpMode() throws InterruptedException {

        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        carousel = hardwareMap.get(DcMotor.class, "carousel");

//        right.setDirection(DcMotorSimple.Direction.REVERSE);
//        left.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        waitForStart();

//        while(opModeIsActive()) {
//            angle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
//            telemetry.addData("angly boi", angle);
//            telemetry.update();
//        }
        drive(8*12*TICKS_PER_INCH);
    }

    public void drive(double distance) {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double lDistance = -left.getCurrentPosition();
        double rDistance = -right.getCurrentPosition();

        double sign = Math.signum(distance);

        while(opModeIsActive() && Math.abs((lDistance + rDistance) / 2) < Math.abs(distance)) {
            double diff = Math.abs(lDistance) - Math.abs(rDistance);

            double leftPower = 0.75;
            double rightPower = 0.75;

            if (diff > 0) leftPower = 0.5;
            else rightPower = 0.5;

            left.setPower(leftPower * sign);
            right.setPower(rightPower * sign);

            lDistance = -left.getCurrentPosition();
            rDistance = -right.getCurrentPosition();

            telemetry.addData("left distance", lDistance);
            telemetry.addData("right distance", rDistance);
            telemetry.addData("angle", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }

    public void turn(double desiredAngle) {
        double angle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double sign = Math.signum(desiredAngle - angle);
        double kP = (desiredAngle-angle) * 0.02;
        while(opModeIsActive() && Math.signum(desiredAngle - angle) == sign) {
//            kP = (desiredAngle-angle) * 0.01675;
//            kP = (desiredAngle-angle) * 0.019;
            left.setPower(-0.4 /* * kP */* sign);
            right.setPower(0.4 /* * kP */* sign);
            angle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            telemetry.addData("angle", angle);
            telemetry.addData("desiredAngle", desiredAngle);
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }
}
