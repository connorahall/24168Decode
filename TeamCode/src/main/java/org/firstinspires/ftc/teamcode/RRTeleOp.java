package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RRTeleOp")
public class RRTeleOp extends OpMode {

    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    private double speed_factor = 0.4;


    @Override
    public void init() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
    }

    @Override
    public void loop() {
        telemetry.update();
        if (gamepad1.right_trigger > 0.5)
            speed_factor = 1;
        else
            speed_factor = 0.4;

        // Drive
        FL.setPower(-(gamepad1.left_stick_y * speed_factor) + (gamepad1.right_stick_x * speed_factor) + (gamepad1.left_stick_x * 0.6));
        FR.setPower((gamepad1.left_stick_y * speed_factor) + (gamepad1.right_stick_x * speed_factor) + (gamepad1.left_stick_x * 0.6));
        BL.setPower((gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) + (gamepad1.left_stick_x * 0.6));
        BR.setPower(-(gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) + (gamepad1.left_stick_x * 0.6));


    }

}