package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "RRTeleOp")
public class RRTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor FL = hardwareMap.get(DcMotor.class, "FL");
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR");
        DcMotor BL = hardwareMap.get(DcMotor.class, "BL");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        double speed_factor = 0.6;


        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            telemetry.update();

            double headingRobot = drive.getPoseEstimate().getHeading();

            if (gamepad1.right_trigger > 0.5)
                speed_factor = 1;
            else
                speed_factor = 0.6;

            // drive code
            Vector2d input;
            input = new Vector2d(
                    -gamepad1.left_stick_y * speed_factor,
                    -gamepad1.left_stick_x * speed_factor
            ).rotated(-headingRobot);

            drive.setWeightedDrivePower(new Pose2d(
                            input.getX(), input.getY(),
                            -gamepad1.right_stick_x * speed_factor
                    )
            );

            // reset robot heading
            if (gamepad1.options){
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0));
            }

//        // Drive before roadrunner implementation
//        FL.setPower((gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) - (gamepad1.left_stick_x * speed_factor));
//        FR.setPower(-(gamepad1.left_stick_y * speed_factor) -  (gamepad1.right_stick_x * speed_factor) - (gamepad1.left_stick_x * speed_factor));
//        BL.setPower((gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) + (gamepad1.left_stick_x * speed_factor));
//        BR.setPower(-(gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) + (gamepad1.left_stick_x * speed_factor));

            drive.update();
        }
    }

}