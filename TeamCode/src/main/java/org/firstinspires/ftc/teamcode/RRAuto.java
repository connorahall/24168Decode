package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RRAuto")
public class RRAuto extends LinearOpMode {

    // different states used to control logic flow in loop
    enum State {
        IDLE, SCORE_FIRST, SCORING_FIRST, PREP
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
        TrajectorySequence scoreFirst, prep;

        if (bot.getTeam() == Robot.Color.BLUE) {
            scoreFirst = drive.trajectorySequenceBuilder(new Pose2d(-50, -50, Math.toRadians(50)))
                    .lineToLinearHeading(new Pose2d(-20, -18, Math.toRadians(230)))
                    .build();

            prep = drive.trajectorySequenceBuilder(scoreFirst.end())
                    .lineToLinearHeading(new Pose2d(-20, -20, Math.toRadians(270)))
                    .build();
        } else {
             scoreFirst = drive.trajectorySequenceBuilder(new Pose2d(-50, 50, Math.toRadians(-40)))
                    .lineToLinearHeading(new Pose2d(-20, 18, Math.toRadians(140)))
                    .build();
             prep = drive.trajectorySequenceBuilder(scoreFirst.end())
                    .lineToLinearHeading(new Pose2d(-20, 20, Math.toRadians(90)))
                    .build();
        }

        bot.setInitialState();

        telemetry.addData("initialization:", "is a success");
        telemetry.addData("WE ARE", bot.getTeam());
        telemetry.update();

        waitForStart();

        bot.setLauncherVelocity(230);

        drive.followTrajectorySequenceAsync(scoreFirst);
        State state = State.SCORE_FIRST;
        int launched = 0;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        if(isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()) {
            bot.cameraTelemetryWhenStill();
            bot.getSorterPID().update();
            dashboardTelemetry.addData("target", bot.getSorterPID().getTarget());
            dashboardTelemetry.addData("actual", bot.getSorterPID().getPosition());
            dashboardTelemetry.update();
            switch(state) {
                case SCORE_FIRST:
                    if (!drive.isBusy()) {
                        state = State.SCORING_FIRST;
                    }
                    break;
                case SCORING_FIRST:
                    bot.autoPowerLauncher();
                    if (launched == 3) {
                        state = State.PREP;
                        drive.followTrajectorySequenceAsync(prep);
                    } else {
                        if (Math.abs(bot.launcher.getVelocity(AngleUnit.DEGREES) - bot.getAutoPower()) < 5) {
//                            System.out.println(bot.launched());
                            bot.launchAndSort();
                            if (bot.launched()) {
                                launched++;
                            }
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