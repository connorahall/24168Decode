package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    enum SorterState {
        INTAKING, LAUNCHING
    }
    State mode = State.OFF;
    SorterState sorterState = SorterState.LAUNCHING;


    Robot bot;
    public static double kp, ki, kd, target;

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


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            telemetry.update();
//            bot.getSorterPID().setKp(kp);
//            bot.getSorterPID().setKi(ki);
//            bot.getSorterPID().setKd(kd);
//            bot.getLauncherPID().setTarget((int)target);


            dashboardTelemetry.addData("intake target", bot.getSorterPID().getTarget());
            dashboardTelemetry.addData("intake actual", -bot.sorter.getCurrentPosition());
            dashboardTelemetry.addData("launch target", bot.getAutoPower());
            dashboardTelemetry.addData("launch actual", bot.launcher.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.update();

            if (gamepad1.dpad_up) {
                if (sorterState == SorterState.INTAKING) {
                    bot.moveSorterCCW(0.5);
                }
                mode = State.LAUNCH;
                sorterState = SorterState.LAUNCHING;
                bot.setIntakePower(0.5);
            } if (gamepad1.dpad_down) {
                if (sorterState == SorterState.LAUNCHING) {
                    bot.moveSorterCCW(0.5);
                }
                mode = State.INTAKE;
                sorterState = SorterState.INTAKING;
                bot.setIntakePower(-1);
                bot.setLauncherVelocity(0);
            } if (gamepad1.dpad_right) {
                mode = State.OFF;
//                bot.getLauncherPID().setPower(0);
                bot.setLauncherVelocity(0);
                bot.setIntakePower(0);
            }

            // adjust power based on position
            if (mode == State.LAUNCH) {
                bot.autoPowerLauncher();
            } else if (mode != State.INTAKE) {
                bot.setLauncherVelocity(0);
            }

            // put a ball into launcher
            if (gamepad1.right_trigger > 0.5) {
                if (gamepad1.dpad_left) {
                    bot.launch();
                } else {
                    bot.launchAndSort();
                }
            }

            // spin drum sorter
            if (gamepad1.left_bumper) {
//                bot.moveSorter(Robot.Color.GREEN);
                bot.moveSorterCCW();
            }
            if (gamepad1.right_bumper) {
//                bot.moveSorter(Robot.Color.PURPLE);
                bot.moveSorterCW();
            }
            if (gamepad1.circle) {
                bot.setTeam(Robot.Color.RED);
                gamepad1.setLedColor(1, 0, 0, 1000);
            }
            if (gamepad1.cross) {
                bot.setTeam(Robot.Color.BLUE);
                gamepad1.setLedColor(0, 0, 1, 1000);

            }

            bot.update();
        }

        bot.getVisionPortal().close();
    }








}