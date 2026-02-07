package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RRAutoFar")
public class RRAutoFar extends LinearOpMode {

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

        for (int i = 0; i < 8; i++) {
            bot.setTeam(bot.identifyMyTeamTeleOp());
            sleep(100);
        }

        for (int i = 0; i < 10; i++) {
            bot.cameraTelemetryWhenStillBypass();
        }

        TrajectorySequence prep, moveForward, score;

        if (bot.getTeam() == Robot.Color.BLUE) {
            prep = drive.trajectorySequenceBuilder(new Pose2d(54, -16, Math.toRadians(207)))
                    .splineToLinearHeading(new Pose2d(36, -30, Math.toRadians(270)), Math.toRadians(270))
                    .build();
            moveForward = drive.trajectorySequenceBuilder(prep.end())
                    .lineToLinearHeading(new Pose2d(36, -52, Math.toRadians(270)),
                            SampleMecanumDrive.getVelocityConstraint(6, 6.5, 12.5),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .build();
            score = drive.trajectorySequenceBuilder(moveForward.end())
                    .lineToLinearHeading(new Pose2d(55, -13, Math.toRadians(207)))
                    .build();
        } else {
            prep = drive.trajectorySequenceBuilder(new Pose2d(54, -16, Math.toRadians(207)))
                    .splineToLinearHeading(new Pose2d(36, -30, Math.toRadians(270)), Math.toRadians(270))
                    .build();
            moveForward = drive.trajectorySequenceBuilder(prep.end())
                    .lineToLinearHeading(new Pose2d(36, -52, Math.toRadians(270)),
                            SampleMecanumDrive.getVelocityConstraint(6, 6.5, 12.5),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .build();
            score = drive.trajectorySequenceBuilder(moveForward.end())
                    .lineToLinearHeading(new Pose2d(54, -16, Math.toRadians(207)))
                    .build();
        }

        bot.setInitialState();

        telemetry.addData("initialization:", "is a success");
        telemetry.addData("WE ARE", bot.getTeam());
        telemetry.update();

        waitForStart();

        bot.launcher.setPower(1);

        State state = State.SCORE;
        drive.followTrajectorySequenceAsync(score);

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
                case SCORING:
                    drive.setWeightedDrivePower(new Pose2d(0, 0, (bot.headingDeviationFromGoalRadians()+Math.toRadians(5))*3));
//                    bot.autoPowerLauncher();
                    bot.launcher.setPower(1);
                    if (count == 3) {
                        if (row == 0) {
                            state = State.PREP;
                            drive.followTrajectorySequenceAsync(prep);
                        } else {
                            state = State.IDLE;
                        }
//                        bot.getLauncherPID().setPower(0);
                        bot.setLauncherVelocity(0);
                    } else {
//                        System.out.println(timer.milliseconds() - time);
                        if (time + 1500 < timer.milliseconds()) {
                            bot.launchAndSort();
                            if (bot.launched()) {
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
                        bot.setIntakePower(-1);
                        drive.followTrajectorySequenceAsync(moveForward);
                        time = timer.milliseconds();
                        count = 0;
                    }
                    break;
                case INTAKING:
                    System.out.println(count);
                    // counts to 4. first count doesn't spin
                    if (count == 0 && time + 400 + (100*row) < timer.milliseconds()) {
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
                    if (count > 0 && time + 750 < timer.milliseconds()) {
                        if (count < 3) {
//                                bot.moveSorterCCW();
                            bot.getSorterPID().setTarget(bot.getSorterPID().getTarget() + 2720);
                            System.out.println("spin");
                        }
                        if (count == 2) {
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
                case SCORE:
                    if (!drive.isBusy()) {
                        state = State.SCORING;
                        bot.setIntakePower(0);
                        if (bot.getSorterPID().getTarget() % 2720 != 0) {
                            bot.moveSorterCCW(0.5);
                        }
                        time = timer.milliseconds();
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
