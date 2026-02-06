package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RRAuto")
public class RRAuto extends LinearOpMode {

    // different states used to control logic flow in loop
    enum State {
        IDLE, SCORE, SCORING, PREP, GET_MOTIF, INTAKING
    }

    // bot object that operates the robot
    Robot bot;
    // used to sync timing of flipper and sorter with bot
    Object lock;


    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        bot = new Robot(drive, hardwareMap);
        lock = bot.getLock();

        System.out.println("1");


        // uses the camera and sets the team to the OPPOSITE of the goal that it can see
        // also sets initial pose
        for (int i = 0; i < 8; i++) {
            bot.setTeam(bot.identifyMyTeamAuto());
            sleep(100);
        }

        System.out.println("2");

        for (int i = 0; i < 10; i++) {
            bot.cameraTelemetryWhenStillBypass();
        }
        System.out.println("3");

        // uses the same trajectories but instantiates them differently depending on our team
        TrajectorySequence getMotif;
        TrajectorySequence[] prep = new TrajectorySequence[3];
        TrajectorySequence[] moveForward = new TrajectorySequence[3];
//        TrajectorySequence[] score = new TrajectorySequence[4];
        TrajectorySequence score;

        if (bot.getTeam() == Robot.Color.BLUE) {

            System.out.println("7");
            getMotif = drive.trajectorySequenceBuilder(bot.getDrive().getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-20, -18, Math.toRadians(160)))
                    .build();
            score = drive.trajectorySequenceBuilder(getMotif.end())
                    .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(227)))
                    .build();
//            score[0] = drive.trajectorySequenceBuilder(getMotif.end())
//                    .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(225)))
//                    .build();
            for (int i = 0; i < 3; i++) {
                System.out.println("8");
                prep[i] = drive.trajectorySequenceBuilder(score.end())
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-12 + (i * 24), -29, Math.toRadians(270)), Math.toRadians(270))
                        .build();
                moveForward[i] = drive.trajectorySequenceBuilder(prep[i].end())
                        .lineToLinearHeading(new Pose2d(-12 + (i * 24), -46, Math.toRadians(270)),
                                SampleMecanumDrive.getVelocityConstraint(6, 6.5, 12.5),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .build();
            }
            System.out.println("9");

        } else {
            getMotif = drive.trajectorySequenceBuilder(bot.getDrive().getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-20, 18, Math.toRadians(200)))
                    .build();
            score = drive.trajectorySequenceBuilder(getMotif.end())
                    .lineToLinearHeading(new Pose2d(-12, 12, Math.toRadians(133)))
                    .build();
//            score[0] = drive.trajectorySequenceBuilder(getMotif.end())
//                    .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(225)))
//                    .build();
            for (int i = 0; i < 3; i++) {
                prep[i] = drive.trajectorySequenceBuilder(score.end())
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-12 + (i * 24), 29, Math.toRadians(90)), Math.toRadians(270))
                        .build();
                moveForward[i] = drive.trajectorySequenceBuilder(prep[i].end())
                        .lineToLinearHeading(new Pose2d(-12 + (i * 24), 46, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(6, 6.5, 12.5),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .build();
            }
        }

        System.out.println("4");

        bot.setInitialState();
//        bot.setPoseEstimate(getMotif.start());



        telemetry.addData("initialization:", "is a success");
        telemetry.addData("WE ARE", bot.getTeam());
        telemetry.update();

        System.out.println("5");

        waitForStart();

//        bot.getLauncherPID().setTarget(200);
        bot.setLauncherVelocity(220);

        drive.followTrajectorySequenceAsync(getMotif);
        State state = State.GET_MOTIF;

//        State state = State.SCORE;
//        drive.followTrajectorySequenceAsync(score[0]);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        System.out.println("6");


        if (isStopRequested()) return;

        ElapsedTime timer = new ElapsedTime();
        double time = 0.0;
        int count = 0;
        int row = 0;

        while (opModeIsActive() && !isStopRequested()) {
            bot.cameraTelemetryWhenStill();

            dashboardTelemetry.addData("target", bot.getAutoPower());
            dashboardTelemetry.addData("actual", bot.launcher.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.update();
            switch (state) {
                case GET_MOTIF:
                    if (bot.motifInt != 0 && !drive.isBusy()) {
                        state = State.SCORE;
                        drive.followTrajectorySequenceAsync(score);
//                        System.out.println(bot.motifInt);
                    }
                    break;
                    case SCORE:
                        if (!drive.isBusy()) {
                            state = State.SCORING;
                            System.out.println("Scoring first");
//                            bot.setIntakePower(0);
                            if (bot.getSorterPID().getTarget() % 2720 != 0) {
                                bot.moveSorterCCW(0.5);
                            }
                            time = timer.milliseconds();
                        }
                        break;
                    case SCORING:
                        if (bot.getTeam() == Robot.Color.BLUE) {
                            drive.setWeightedDrivePower(new Pose2d(0, 0, (bot.headingDeviationFromGoalRadians() + Math.toRadians(2)) * 3));
                        } else {
                            drive.setWeightedDrivePower(new Pose2d(0, 0, (bot.headingDeviationFromGoalRadians() + Math.toRadians(2)) * 3));
                        }
                        bot.autoPowerLauncher();
//                        System.out.println(row);
                        if (count == 3) {
                            if (row < 2) {
                                state = State.PREP;
                                drive.followTrajectorySequenceAsync(prep[row]);
                            } else {
                                state = State.IDLE;
                            }
//                        bot.getLauncherPID().setPower(0);
                            System.out.println("zero");
                            bot.setLauncherVelocity(0);
                        } else {
//                        System.out.println(timer.milliseconds() - time);
                            if (time + 800 < timer.milliseconds()) {
                                bot.launchAndSort();
                                System.out.println("launch?");
                                if (bot.launched()) {
                                    System.out.println("launched?");
//                                if (count == 0) {
//                                    drive.followTrajectorySequenceAsync(score);
//                                }
                                    count++;
                                    bot.setLaunched(false);
                                    time = timer.milliseconds();
                                }
                            }

                        }
                        break;
                    case PREP:
                        if (!drive.isBusy()) {
                            state = State.INTAKING;
                            bot.moveSorterCCW(-0.5);
//                            bot.setIntakePower(-1);
                            drive.followTrajectorySequenceAsync(moveForward[row]);
                            time = timer.milliseconds();
                            count = 0;
                        }
                        break;
                    case INTAKING:
                        System.out.println(count);
                        // counts to 4. first count doesn't spin
                        if (count == 0 && time + 350 + (100*row) < timer.milliseconds()) {
                            time = timer.milliseconds();
                            count++;
                        }
                        // once the intake detects a ball, start the spin timer
//                    if (bot.intake.getVelocity() > -2200) {
//                        time = timer.milliseconds();
//                    }
//                    bot.getSorterPID().setPower(0.2);
                        // spin a certain time after the intake detects a ball
                        // after that, hold until a new ball is detected
                        if (count > 0 && time + 550 < timer.milliseconds()) {
                            System.out.println("spin");
                            if (count < 4) {
                                bot.moveSorterCCW();
                            }
                            if (count == 3) {
                                bot.autoPowerLauncher();
                            }
//                        if (count == 3) {
//                            bot.moveSorterCCW(0.5);
//                        }
                            count++;
                            time = timer.milliseconds();
                            // dont move the sorter while waiting for the next ball to be intaked
//                        time = Integer.MAX_VALUE;
                        }
                        if (!drive.isBusy()) {
                            state = State.SCORE;
                            count = 0;
                            row++;
                            drive.followTrajectorySequenceAsync(score);
                        }
                        break;
                    case IDLE:
                        bot.setInitialState();
                        break;
                }

                drive.update();
                bot.update();
            }

     }

}
