package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RRAuto")
@Disabled
public class RRAuto extends LinearOpMode {

    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    private double speed_factor = 0.4;


    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        telemetry.addData("initialization:", "is a success");
        telemetry.update();

        FL.setTargetPosition(FL.getCurrentPosition());
        BL.setTargetPosition(BL.getCurrentPosition());
        FR.setTargetPosition(FR.getCurrentPosition());
        BR.setTargetPosition(BR.getCurrentPosition());

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();




        // drive forward
        forward(3.5);
        sleep(1000);

        // drive forward
        forward(.3);
        sleep(1000);

        // drive backward
        forward(-1.4);
        sleep(1000);


    }



    public void turnRight(double rotations) {
        move(rotations, -1, -1, -1, -1);
    }
    public void forward(double rotations) {
        move(rotations, -1, -1, 1, 1);
    }

    public void strafe(double rotations) {
        move(rotations, -1, 1, -1, 1);
    }
    public void rotate(double rotations) {
        move(rotations, 1, 1, 1, 1);
    }
    private void move(double rotations, int fl, int bl, int fr, int br) {
        FL.setPower(0.5);
        BL.setPower(0.5);
        FR.setPower(0.5);
        BR.setPower(0.5);
        FL.setTargetPosition(FL.getCurrentPosition() + (int)(rotations * 360 * fl));
        BL.setTargetPosition(BL.getCurrentPosition() + (int)(rotations * 360 * bl));
        FR.setTargetPosition(FR.getCurrentPosition() + (int)(rotations * 360 * fr));
        BR.setTargetPosition(BR.getCurrentPosition() + (int)(rotations * 360 * br));

    }

}