package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class PIDAuto extends LinearOpMode {
    private DcMotor left;
    private DcMotor right;
    private DcMotor intake;

    private static double TICKS_PER_INCH = 651.897;
    private static int RADIUS = 2;
    private int leftTarget;
    private int rightTarget;
    private double speed = 0.9;
    private double kP = 0.001;

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        intake = hardwareMap.get(DcMotor.class, "intake");
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        speed = 0.5;
        leftTarget = 0;
        rightTarget = 0;

        waitForStart();

        drive(left, right, 10000);

    }

    public void drive(DcMotor left, DcMotor right, int setpoint) {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int error = setpoint - ((left.getCurrentPosition() + right.getCurrentPosition()) / 2);

        left.setPower(error * kP);
        right.setPower(error * kP);
    }
}
