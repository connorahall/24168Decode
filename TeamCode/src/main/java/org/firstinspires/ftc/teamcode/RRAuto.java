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


        // uses the camera and sets the team to the OPPOSITE of the goal that it can see
        // also sets initial pose
        for (int i = 0; i < 8; i++) {
            bot.setTeam(bot.identifyMyTeamAuto());
            sleep(100);
        }

        // uses the same trajectories but instantiates them differently depending on our team
        TrajectorySequence getMotif;
        TrajectorySequence[] prep = new TrajectorySequence[3];
        TrajectorySequence[] moveForward = new TrajectorySequence[3];
        TrajectorySequence[] score = new TrajectorySequence[4];

        if (bot.getTeam() == Robot.Color.BLUE) {
            getMotif = drive.trajectorySequenceBuilder(bot.getDrive().getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-20, -18, Math.toRadians(160)))
                    .build();
            score[0] = drive.trajectorySequenceBuilder(getMotif.end())
                    .lineToLinearHeading(new Pose2d(getMotif.end().getX()+0.1, getMotif.end().getY()+0.1, Math.toRadians(225)))
                    .build();
            prep[0] = drive.trajectorySequenceBuilder(score[0].end())
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-12, -34, Math.toRadians(270)), Math.toRadians(270))
                    .build();
            moveForward[0] = drive.trajectorySequenceBuilder(prep[0].end())
                    .lineToLinearHeading(new Pose2d(-12, -52, Math.toRadians(270)),
                            SampleMecanumDrive.getVelocityConstraint(10, 6.5, 12.5),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .build();
            score[1] = drive.trajectorySequenceBuilder(moveForward[0].end())
                    .lineToLinearHeading(new Pose2d(-15, -13, Math.toRadians(225)))
                    .build();
            prep[1] = drive.trajectorySequenceBuilder(score[1].end())
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(12, -34, Math.toRadians(270)), Math.toRadians(270))
                    .build();
            moveForward[1] = drive.trajectorySequenceBuilder(prep[1].end())
                    .lineToLinearHeading(new Pose2d(12, -52, Math.toRadians(270)),
                            SampleMecanumDrive.getVelocityConstraint(10, 6.5, 12.5),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .build();
            score[2] = drive.trajectorySequenceBuilder(moveForward[1].end())
                    .lineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(225)))
                    .build();
            prep[2] = drive.trajectorySequenceBuilder(score[2].end())
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(36, -34, Math.toRadians(270)), Math.toRadians(270))
                    .build();
            moveForward[2] = drive.trajectorySequenceBuilder(prep[2].end())
                    .lineToLinearHeading(new Pose2d(36, -52, Math.toRadians(270)),
                            SampleMecanumDrive.getVelocityConstraint(10, 6.5, 12.5),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .build();
            score[3] = drive.trajectorySequenceBuilder(moveForward[2].end())
                    .lineToLinearHeading(new Pose2d(getMotif.end().getX()+0.1, getMotif.end().getY()+0.1, Math.toRadians(230)))
                    .build();

        } else {
            getMotif = drive.trajectorySequenceBuilder(new Pose2d(-50, -50, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(-20, -18, Math.toRadians(160)))
                    .build();
            score[0] = drive.trajectorySequenceBuilder(getMotif.end())
                    .lineToLinearHeading(new Pose2d(getMotif.end().getX()+0.1, getMotif.end().getY()+0.1, Math.toRadians(230)))
                    .build();
            prep[0] = drive.trajectorySequenceBuilder(score[0].end())
                    .lineToLinearHeading(new Pose2d(-12, -34, Math.toRadians(270)))
                    .build();
            moveForward[0] = drive.trajectorySequenceBuilder(prep[0].end())
                    .lineToLinearHeading(new Pose2d(-12, -52, Math.toRadians(270)),
                            SampleMecanumDrive.getVelocityConstraint(10, 6.5, 12.5),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .build();
            score[1] = drive.trajectorySequenceBuilder(moveForward[0].end())
                    .lineToLinearHeading(new Pose2d(getMotif.end().getX()+0.1, getMotif.end().getY()+0.1, Math.toRadians(230)))
                    .build();
            prep[1] = drive.trajectorySequenceBuilder(score[1].end())
                    .lineToLinearHeading(new Pose2d(12, -34, Math.toRadians(270)))
                    .build();
            moveForward[1] = drive.trajectorySequenceBuilder(prep[1].end())
                    .lineToLinearHeading(new Pose2d(12, -52, Math.toRadians(270)),
                            SampleMecanumDrive.getVelocityConstraint(10, 6.5, 12.5),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .build();
            score[2] = drive.trajectorySequenceBuilder(moveForward[1].end())
                    .lineToLinearHeading(new Pose2d(getMotif.end().getX()+0.1, getMotif.end().getY()+0.1, Math.toRadians(230)))
                    .build();
            prep[2] = drive.trajectorySequenceBuilder(score[2].end())
                    .lineToLinearHeading(new Pose2d(36, -34, Math.toRadians(270)))
                    .build();
            moveForward[2] = drive.trajectorySequenceBuilder(prep[2].end())
                    .lineToLinearHeading(new Pose2d(36, -52, Math.toRadians(270)),
                            SampleMecanumDrive.getVelocityConstraint(10, 6.5, 12.5),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .build();
            score[3] = drive.trajectorySequenceBuilder(moveForward[2].end())
                    .lineToLinearHeading(new Pose2d(getMotif.end().getX()+0.1, getMotif.end().getY()+0.1, Math.toRadians(230)))
                    .build();
        }
//        bot.setPoseEstimate(getMotif.start());

        for (int i = 0; i < 10; i++) {
            bot.cameraTelemetryWhenStill();
        }

        bot.setInitialState();

        telemetry.addData("initialization:", "is a success");
        telemetry.addData("WE ARE", bot.getTeam());
        telemetry.update();

        waitForStart();

        bot.getLauncherPID().setTarget(200);

        drive.followTrajectorySequenceAsync(getMotif);
        State state = State.GET_MOTIF;

//        State state = State.SCORE;
//        drive.followTrajectorySequenceAsync(score[0]);



        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        if(isStopRequested()) return;

        ElapsedTime timer = new ElapsedTime();
        double time = 0.0;
        int count = 0;
        int row = 0;

        while(opModeIsActive() && !isStopRequested()) {
            bot.cameraTelemetryWhenStill();

            dashboardTelemetry.addData("target", bot.getAutoPower());
            dashboardTelemetry.addData("actual", bot.launcher.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.update();
            switch(state) {
                case GET_MOTIF:
                    if (bot.motifInt != 0 && !drive.isBusy()) {
                        state = State.SCORE;
                        drive.followTrajectorySequenceAsync(score[row]);
//                        System.out.println(bot.motifInt);
                    }
                    break;
                case SCORE:
                    if (!drive.isBusy()) {
                        state = State.SCORING;
                        System.out.println("Scoring first");
                        bot.setIntakePower(0);
                        if (bot.getSorterPID().getTarget() % 2720 != 0) {
                            bot.moveSorterCCW(0.5);
                        }
                    }
                    break;
                case SCORING:
                    bot.autoPowerLauncher();
                    System.out.println(row);
                    if (count == 3) {
                        if (row < 3) {
                            state = State.PREP;
                            drive.followTrajectorySequenceAsync(prep[row]);
                        } else {
                            state = State.IDLE;
                        }
                        bot.getLauncherPID().setPower(0);
                    } else {
//                        System.out.println(timer.milliseconds() - time);
                        if (time + 1000 < timer.milliseconds()) {

//                            System.out.println(bot.launched());
//                            if (bot.drum[0] == bot.motif[launched]) {
//                                bot.launch();
//                            } else if (bot.drum[1] == bot.motif[launched]) {
//                                bot.moveSorterCW();
//                            } else if (bot.drum[2] == bot.motif[launched]) {
//                                bot.moveSorterCCW();
//                            } else {
//                                bot.launch();
//                                System.out.println("boo");
//                            }
                            bot.launchAndSort();
                            if (bot.launched()) {
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
                        bot.moveSorterCCW(0.5);
                        bot.setIntakePower(-1);
                        drive.followTrajectorySequenceAsync(moveForward[row]);
                        time = timer.milliseconds();
                        count = 0;
                    }
                    break;
                case INTAKING:
                    System.out.println(count);
                    // counts to 4. first count doesn't spin
                    if (count == 0 && time + 500 + (100*row) < timer.milliseconds()) {
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
                    if (time + 400 < timer.milliseconds()) {
                        if (count < 4 && count > 0) {
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
                        drive.followTrajectorySequenceAsync(score[row]);
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