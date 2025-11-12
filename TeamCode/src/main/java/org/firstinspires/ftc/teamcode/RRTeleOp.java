package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
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
    enum State {
        INTAKE, LAUNCH
    }
    State mode = State.LAUNCH;

    Robot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx FL = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx FR = hardwareMap.get(DcMotorEx.class, "FR");
        DcMotorEx BL = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx BR = hardwareMap.get(DcMotorEx.class, "BR");
        Servo flipper = hardwareMap.get(Servo.class, "flipper");
        CRServo sorter = hardwareMap.get(CRServo.class, "sorter");

        bot = new Robot(launcher, intake, FL, FR, BL, BR, flipper, sorter, Robot.OpMode.TELEOP);

//        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        bot.getDrive().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));


//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        drive.setPoseEstimate(new Pose2d(0, 0, 0));

//        double launchTablePrecision = 0.01;
//        HashMap<Double, Double> launchTable = new HashMap<>((int) (1.5 / launchTablePrecision));
//        launchTable = generateLaunchTable(launchTablePrecision);

        telemetry.addData("initialization:", "is a success");
        telemetry.update();
        waitForStart();

        bot.setInitialState();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            telemetry.update();

            if (gamepad1.dpad_up) {
                mode = State.LAUNCH;
                bot.setLauncherVelocity(230);
                bot.setIntakePower(0.5);
            } if (gamepad1.dpad_down) {
                mode = State.INTAKE;
                bot.setLauncherVelocity(0);
                bot.setIntakePower(-1);
            } if (gamepad1.dpad_right) {
                bot.setLauncherVelocity(0);
                bot.setIntakePower(0);
            }

            // put a ball into launcher
            if (gamepad1.right_trigger > 0.5) {
                bot.launch();
            }

            // drum sorter spinner code
            if (gamepad1.left_bumper) {
                bot.moveSorter();
            }

            if (gamepad1.right_bumper) {
                bot.moveSorterReverse();
            }

            bot.update();
        }
    }

    HashMap<Double, Double> generateLaunchTable(double precision) {

        HashMap<Double, Double> ret = new HashMap<>((int) (1.5 / precision));

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