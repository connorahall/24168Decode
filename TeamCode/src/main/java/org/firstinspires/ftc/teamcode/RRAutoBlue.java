package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name = "RRAutoBlue")
public class RRAutoBlue extends LinearOpMode {

    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;

    enum State {
        IDLE, SCORE_FIRST, SCORING_FIRST, PREP
    }

    ElapsedTime game;



    public void runOpMode() {


        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        Servo flipper = hardwareMap.get(Servo.class, "flipper");
        CRServo sorter = hardwareMap.get(CRServo.class, "sorter");
        telemetry.addData("initialization:", "is a success");
        telemetry.update();


        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        TrajectorySequence scoreFirst = drive.trajectorySequenceBuilder(new Pose2d(-50, -50, Math.toRadians(50)))
                .lineToLinearHeading(new Pose2d(-20, -18, Math.toRadians(230)))
                .build();
        TrajectorySequence prep = drive.trajectorySequenceBuilder(scoreFirst.end())
                .lineToLinearHeading(new Pose2d(-20, -20, Math.toRadians(270)))
                .build();


        drive.setPoseEstimate(scoreFirst.start());

//        System.out.println("build");

        waitForStart();

//        System.out.println("start");

        double flipperTime = 0.0;
        boolean flipping = false;
        double spinningTime = 0.0;
        boolean spinning = false;

        ElapsedTime timer = new ElapsedTime();

        intake.setPower(0.5);
        sorter.setPower(0);
        flipper.setPosition(0);
        launcher.setVelocity(220, AngleUnit.DEGREES);


        drive.followTrajectorySequenceAsync(scoreFirst);
        State state = State.SCORE_FIRST;
        int launched = 0;

        game = new ElapsedTime();


        if(isStopRequested()) return;

//        System.out.println("before loop");

        while(opModeIsActive() && !isStopRequested()) {
//            System.out.println(game.milliseconds());

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
                        if (!flipping && !spinning) {
                            flipping = true;
                            flipperTime = game.milliseconds();
                            flipper.setPosition(1);
//                            System.out.println("first");
                        } else if (flipping && flipperTime + 1000 < game.milliseconds()) {
                            flipper.setPosition(0);
                            flipping = false;
                            launched++;
                            System.out.println(launcher.getVelocity(AngleUnit.DEGREES));
                            spinning = true;
                            spinningTime = game.milliseconds();
//                            System.out.println("second");
                        }
                        else if (spinning && spinningTime + 1000 < game.milliseconds() && spinningTime + 1050 > game.milliseconds()) {
                            sorter.setPower(-1);
//                            System.out.println("third");
                        }
                        else if (spinning && spinningTime + 1502 < game.milliseconds()) {
                            sorter.setPower(0);
                            spinning = false;
//                            System.out.println("Fourth");
                        }

                    }
                    break;
                case PREP:
                    if (!drive.isBusy()) {
                        state = State.IDLE;
                    }
                    break;
                case IDLE:
                    launcher.setPower(0);
                    sorter.setPower(0);
                    flipper.setPosition(0);
                    intake.setPower(0);
                    break;
            }

            drive.update();
        }




    }

}