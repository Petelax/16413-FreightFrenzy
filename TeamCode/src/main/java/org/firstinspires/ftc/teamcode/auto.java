package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class auto extends LinearOpMode {
    private DcMotor left;
    private DcMotor right;
    private DcMotor intake;

    private static double TICKS_PER_INCH = 651.897;
    private static int RADIUS = 2;
    private int leftTarget;
    private int rightTarget;
    private double speed = 0.9;

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        intake = hardwareMap.get(DcMotor.class, "intake");
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        left.setDirection(DcMotorSimple.Direction.REVERSE);
//        right.setDirection(DcMotorSimple.Direction.REVERSE);

        speed = 0.5;
        leftTarget = 0;
        rightTarget = 0;

        waitForStart();

        drive(left, right, 24);
    }

    public void drive(DcMotor left, DcMotor right, int position) {
        telemetry.addData("Drivin", "");
        telemetry.update();
        int target = (int) (position * TICKS_PER_INCH);

        leftTarget = leftTarget + target;
        rightTarget = rightTarget + target;

        left.setTargetPosition(-leftTarget);
        right.setTargetPosition(-rightTarget);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(speed);
        right.setPower(speed);
        while (left.isBusy() && right.isBusy()) {
            telemetry.addData("left", left.getCurrentPosition());
            telemetry.addData("right", right.getCurrentPosition());
            telemetry.update();
        }

    }

    private void turn(double rotation, DcMotor left, DcMotor right) {
        int target = (int) ((rotation / 180) * Math.PI * RADIUS * TICKS_PER_INCH);

        leftTarget = leftTarget + target;
        rightTarget = rightTarget - target;

        left.setTargetPosition(leftTarget);
        right.setTargetPosition(rightTarget);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(speed);
        right.setPower(-speed);

        while (left.isBusy() && right.isBusy()) {
            telemetry.addData("Error", target - ((left.getCurrentPosition() + right.getCurrentPosition()) / 2));
            telemetry.update();
        }
    }
}