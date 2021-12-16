package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestOpMode extends OpMode {
    private DcMotor left;
    private DcMotor right;
    private DcMotor intake;
    private DcMotor intake1;
    private DcMotor carousel;
    private DcMotor elevator0;
    private DcMotor elevator1;
    private Servo hold;
    private CRServo bucket;

    @Override
    public void init() {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        elevator1 = hardwareMap.get(DcMotor.class, "elevator0");
        elevator0 = hardwareMap.get(DcMotor.class, "elevator1");
        hold = hardwareMap.get(Servo.class, "hold");
        bucket = hardwareMap.get(CRServo.class, "bucket");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void loop() {
        double speed = gamepad1.right_trigger - gamepad1.left_trigger;
        double turn;
        double intakeSpeed = (gamepad2.a ? 0.9 : 0) + (gamepad2.b ? -0.9 : 0);
        double elevatorSpeed = (gamepad2.dpad_up ? 0.8 : 0) + (gamepad2.dpad_down ? -0.8 : 0);
        if(gamepad1.left_stick_x >= 0.95 || gamepad1.left_stick_x <= -0.95) {
            turn = gamepad1.left_stick_x;
        } else {
            turn = gamepad1.left_stick_x / 1.5;
        }
        double axleDifference = left.getCurrentPosition() - right.getCurrentPosition();

        telemetry.addData("speed", speed);
        telemetry.addData("turn", turn);
        telemetry.addData("left", left.getCurrentPosition());
        telemetry.addData("right", right.getCurrentPosition());

        intake.setPower(intakeSpeed);
        intake1.setPower(intakeSpeed);
        carousel.setPower(gamepad2.left_stick_x);
        hold.setPosition((gamepad2.right_stick_x+1)*90);
        bucket.setPower((gamepad2.x ? 0.9 : 0) + (gamepad2.y ? -0.9 : 0));
        elevator0.setPower(elevatorSpeed);
        elevator1.setPower(-elevatorSpeed);

        if(turn != 0) {
            left.setPower(speed + turn);
            right.setPower(speed - turn);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if(speed != 0){
            left.setPower(speed + (axleDifference/10000));
            right.setPower(speed - (axleDifference/10000));
        }
        else{
            left.setPower(0);
            right.setPower(0);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
