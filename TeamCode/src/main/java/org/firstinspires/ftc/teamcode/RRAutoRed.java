package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "RRAutoRed")
public class RRAutoRed extends LinearOpMode {

    enum State {
        IDLE, SCORE_FIRST, SCORING_FIRST, PREP
    }

    Robot bot;
//    final Object lock = bot.getLock();
    Object lock;


    public void runOpMode() throws InterruptedException {

        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx FL = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx FR = hardwareMap.get(DcMotorEx.class, "FR");
        DcMotorEx BL = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx BR = hardwareMap.get(DcMotorEx.class, "BR");
        Servo flipper = hardwareMap.get(Servo.class, "flipper");
        CRServo sorter = hardwareMap.get(CRServo.class, "sorter");
        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "sensor");


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        bot = new Robot(drive, launcher, intake, FL, FR, BL, BR, flipper, sorter, sensor);
        lock = bot.getLock();

//
//        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        TrajectorySequence scoreFirst = drive.trajectorySequenceBuilder(new Pose2d(-50, 50, Math.toRadians(-40)))
                .lineToLinearHeading(new Pose2d(-20, 18, Math.toRadians(140)))
                .build();
        TrajectorySequence prep = drive.trajectorySequenceBuilder(scoreFirst.end())
                .lineToLinearHeading(new Pose2d(-20, 20, Math.toRadians(90)))
                .build();


        bot.setPoseEstimate(scoreFirst.start());

        telemetry.addData("initialization:", "is a success");
        telemetry.update();

        waitForStart();

        bot.setInitialState();
        bot.setLauncherVelocity(230);

        drive.followTrajectorySequenceAsync(scoreFirst);
        State state = State.SCORE_FIRST;
        int launched = 0;

        if(isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()) {
            switch(state) {
                case SCORE_FIRST:
                    if (!drive.isBusy()) {
                        state = State.SCORING_FIRST;
                    }
                    break;
                case SCORING_FIRST:
                    if (launched == 3) {
                        state = State.PREP;
                        drive.followTrajectorySequenceAsync(prep);
                    } else {
                        synchronized (lock) {
                            bot.launchAndSort();
                            wait();
                            launched++;
                        }
                    }
                    break;
                case PREP:
                    if (!drive.isBusy()) {
                        state = State.IDLE;
                    }
                    break;
                case IDLE:
                    bot.setRestState();
                    break;
            }

            drive.update();
            bot.update();
        }


    }

}