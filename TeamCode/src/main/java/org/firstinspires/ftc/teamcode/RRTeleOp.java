package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;


@TeleOp(name = "RRTeleOp")
@Config
public class RRTeleOp extends LinearOpMode {




    SampleMecanumDrive drive;


    ElapsedTime game;
    enum State {
        INTAKE, LAUNCH, OFF
    }
    State mode = State.OFF;


    Robot bot;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        bot = new Robot(drive, gamepad1, hardwareMap, Robot.OpMode.TELEOP);
        bot.setTeam(Robot.Color.EMPTY);

        bot.getDrive().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

        telemetry.addData("initialization:", "is a success");
        telemetry.update();
        waitForStart();

        bot.setInitialState();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            telemetry.update();


            if (gamepad1.dpad_up) {
//                bot.getVisionPortal().resumeStreaming();
                mode = State.LAUNCH;
//                bot.setLauncherVelocity(vel);
                bot.setIntakePower(-0.5);
            } if (gamepad1.dpad_down) {
//                bot.getVisionPortal().stopStreaming();
                mode = State.INTAKE;
                bot.setLauncherVelocity(0);
                bot.setIntakePower(1);
            } if (gamepad1.dpad_right) {
//                bot.getVisionPortal().stopStreaming();
                mode = State.OFF;
                bot.setLauncherVelocity(0);
                bot.setIntakePower(0);
            }

            // adjust power based on position
            if (mode == State.LAUNCH) {
                bot.autoPowerLauncher();
            } else {
                bot.setLauncherVelocity(0);
            }

            // put a ball into launcher
            if (gamepad1.right_trigger > 0.5) {
                if (bot.getTeam() == Robot.Color.EMPTY) {
                    bot.setTeam(bot.identifyMyTeamTeleOp());
                }
                else {
                    bot.launch();
                }
            }

            // spin drum sorter
            if (gamepad1.left_bumper) {
                bot.moveSorter(Robot.Color.GREEN);
            }
            if (gamepad1.right_bumper) {
                bot.moveSorter(Robot.Color.PURPLE);
            }

            bot.update();
        }

        bot.getVisionPortal().close();
    }








}