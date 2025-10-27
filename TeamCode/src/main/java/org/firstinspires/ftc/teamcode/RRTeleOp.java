package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.HashMap;

@TeleOp(name = "RRTeleOp")
public class RRTeleOp extends LinearOpMode {


    // units in inches and seconds
    // launch velocity
    double v0 = 10;
    // gravity
    double a = 386.1;
    // height from launcher to goal
    double h = 40;
    ElapsedTime game;



    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor FL = hardwareMap.get(DcMotor.class, "FL");
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR");
        DcMotor BL = hardwareMap.get(DcMotor.class, "BL");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR");
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        Servo flipper = hardwareMap.get(Servo.class, "flipper");
        CRServo sorter = hardwareMap.get(CRServo.class, "sorter");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        double speed_factor;

        waitForStart();

        // 6000 rpm = 36000 deg/s
//        launcher.setVelocity(36000, AngleUnit.DEGREES);

        double launchTablePrecision = 0.01;
        HashMap<Double, Double> launchTable = new HashMap<Double, Double>((int)(1.5 / launchTablePrecision));
        launchTable = generateLaunchTable(launchTablePrecision);

        double flipperTime = 0.0;
        boolean flipping = false;
        double spinningTime = 0.0;
        boolean spinning = false;

        ElapsedTime timer = new ElapsedTime();

        game = new ElapsedTime();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            telemetry.update();

            double headingRobot = drive.getPoseEstimate().getHeading();

            if (gamepad1.left_trigger > 0.5)
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

            // put a ball into launcher
            if (gamepad1.right_trigger > 0.5) {
                flipping = true;
                flipperTime = game.milliseconds();
                flipper.setPosition(1);
            }
            if (flipping && flipperTime + 500 < game.milliseconds()) {
                flipper.setPosition(0);
                flipping = false;
            }

            // drum sorter spinner code
            if (!spinning) {
                spinningTime = game.milliseconds();
            }
            if (gamepad1.left_bumper && !spinning) {
                spinning = true;
                sorter.setPower(-1);
            }
            if (spinning && spinningTime + 498 < game.milliseconds()) {
                sorter.setPower(0);
                if (!gamepad1.left_bumper || !gamepad1.right_bumper)
                    spinning = false;
            }
            if (gamepad1.right_bumper && !spinning) {
                spinning = true;
                sorter.setPower(1);
            }
            if (spinning && spinningTime + 498 < game.milliseconds()) {
                sorter.setPower(0);
                if (!gamepad1.left_bumper || !gamepad1.right_bumper)
                    spinning = false;
            }

            System.out.println(launcher.getVelocity(AngleUnit.DEGREES));
//            launcher.setPower(1);
//            launcher.setVelocity((int)(36000. * gamepad1.right_stick_y), AngleUnit.DEGREES);



//        // Drive before roadrunner implementation
//        FL.setPower((gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) - (gamepad1.left_stick_x * speed_factor));
//        FR.setPower(-(gamepad1.left_stick_y * speed_factor) -  (gamepad1.right_stick_x * speed_factor) - (gamepad1.left_stick_x * speed_factor));
//        BL.setPower((gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) + (gamepad1.left_stick_x * speed_factor));
//        BR.setPower(-(gamepad1.left_stick_y * speed_factor) - (gamepad1.right_stick_x * speed_factor) + (gamepad1.left_stick_x * speed_factor));

            drive.update();
        }
    }

    HashMap<Double, Double> generateLaunchTable(double precision) {

        HashMap<Double, Double> ret = new HashMap<Double, Double>((int)(1.5 / precision));

        // find max of angle to distance
        double max = 0.;
        while (ddxAngleToDistance(max) > 0) {
            max += precision;
        }

        // generate table
        // Map distance to the required angle
        double distance;
        for (double i = max; i < Math.toRadians(90); i+=precision) {
            distance = Math.round((angleToDistance(i) / precision)) * precision;
            ret.put(distance, i);
        }

        return ret;

    }

    double angleToDistance(double angle) {

        return (v0 / a) * (Math.cos(angle) * (-1*Math.sin(angle)*v0 - Math.sqrt(v0*v0*Math.sin(angle)*Math.sin(angle)+(2*a*h))));
    }

    double ddxAngleToDistance(double angle) {
        return (v0 / a) * (Math.sin(angle) * (Math.sin(angle)*v0 + Math.sqrt(v0*v0*Math.sin(angle)*Math.sin(angle) + 2*a*h)) - Math.cos(angle)*(Math.cos(angle)*v0+(2*v0*v0*Math.sin(angle)*Math.cos(angle)) / (2*Math.sqrt(v0*v0*Math.sin(angle)*Math.sin(angle) + 2*a*h))));
    }

}
