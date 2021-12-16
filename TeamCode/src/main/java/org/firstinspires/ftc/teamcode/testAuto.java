package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class testAuto extends LinearOpMode {
    private DcMotor left;
    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "left");
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("left", left.getCurrentPosition());
            left.setPower(0.5);
            telemetry.update();
        }
    }
}
