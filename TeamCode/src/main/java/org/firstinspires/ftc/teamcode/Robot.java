package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;
import java.util.Map;

public class Robot {
    SampleMecanumDrive drive;
    Gamepad gamepad1;
    DcMotorEx launcher, intake, FL, FR, BL, BR;
    Servo flipper;
    CRServo sorter;
    RevColorSensorV3 sensor;
    double flipperTime;
    boolean flipping;
    double spinningTime;
    boolean spinning;
    boolean flippingAndSpinning;
    double speedFactor;
    ElapsedTime timer;
    private final Object lock;
    public enum OpMode {
        AUTO, TELEOP
    }
    public enum Color {
        PURPLE, GREEN, EMPTY
    }
    Color[] drum = {Color.EMPTY, Color.EMPTY, Color.EMPTY};

    OpMode mode = OpMode.AUTO;

    public Robot(SampleMecanumDrive drive, DcMotorEx launcher, DcMotorEx intake, DcMotorEx FL, DcMotorEx FR, DcMotorEx BL, DcMotorEx BR, Servo flipper, CRServo sorter, RevColorSensorV3 sensor) {
        this.launcher = launcher;
        this.intake = intake;
        this.FL = FL;
        this.FR = FR;
        this.BL = BL;
        this.BR = BR;
        this.flipper = flipper;
        this.sorter = sorter;
        this.sensor = sensor;

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.drive = drive;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flipperTime = 0.0;
        flipping = false;
        spinningTime = 0.0;
        spinning = false;
        flippingAndSpinning = false;

        lock = new Object();

        timer = new ElapsedTime();
    }

    // auto constructor
    public Robot(SampleMecanumDrive drive, DcMotorEx launcher, DcMotorEx intake, DcMotorEx FL, DcMotorEx FR, DcMotorEx BL, DcMotorEx BR, Servo flipper, CRServo sorter, RevColorSensorV3 sensor, Pose2d startingPose) {
        this(drive, launcher, intake, FL, FR, BL, BR, flipper, sorter, sensor);
        drive.setPoseEstimate(startingPose);
    }

    // teleop constructor
    public Robot(SampleMecanumDrive drive, Gamepad gamepad, DcMotorEx launcher, DcMotorEx intake, DcMotorEx FL, DcMotorEx FR, DcMotorEx BL, DcMotorEx BR, Servo flipper, CRServo sorter, RevColorSensorV3 sensor, Robot.OpMode mode) {
        this(drive, launcher, intake, FL, FR, BL, BR, flipper, sorter, sensor);
        this.mode = mode;
        gamepad1 = gamepad;
    }


    public void update() throws InterruptedException {

//        System.out.println("0: " + drum[0] + " 1: " + drum[1] + " 2: " + drum[2]);
//        System.out.println(readColor());
//        System.out.println(launcher.getVelocity(AngleUnit.DEGREES));

        if (flippingAndSpinning) {
            launchingAndSorting();
        }
        if (flipping) {
            launching();
        }
        if (spinning) {
            sorting();
        }

        if (mode == OpMode.TELEOP) {

            if (gamepad1.left_trigger > 0.5)
                speedFactor = 1;
            else
                speedFactor = 0.6;

            if (!(spinning || gamepad1.left_bumper || gamepad1.right_bumper)) {
                sorter.setPower(0.15 * gamepad1.right_stick_y);
            }

            Vector2d input;
            input = new Vector2d(-gamepad1.left_stick_y * speedFactor, -gamepad1.left_stick_x * speedFactor)
                    .rotated(-(drive.getPoseEstimate().getHeading()-Math.toRadians(90)));

            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x * speedFactor));

            if (gamepad1.options){
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 90));
            }

            drive.update();
        }
    }

    public SampleMecanumDrive getDrive() {
        return drive;
    }
    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }
    public Object getLock() {
        return lock;
    }
    public void setOpMode(OpMode mode) {
        this.mode = mode;
    }
    public void setInitialState() {
        intake.setPower(0);
        sorter.setPower(0);
        flipper.setPosition(0);

    }
    public void setRestState() {
        launcher.setPower(0);
        sorter.setPower(0);
        flipper.setPosition(0);
        intake.setPower(0);
        drum[0] = readColor();
    }
    public void setLauncherVelocity(int vel) {
        launcher.setVelocity(vel, AngleUnit.DEGREES);
    }
    public void setIntakePower(double power) {
        intake.setPower(power);
    }
    public void launch() {
        if (!flipping) {
            flipping = true;
            flipperTime = timer.milliseconds();
            flipper.setPosition(1);
        }
    }
    private void launching() {
        if (flipperTime + 500 < timer.milliseconds()) {
            flipper.setPosition(0);
            flipping = false;
            drum[0] = readColor();
        }
    }
    public void moveSorter() {
        if (!spinning) {
            spinning = true;
            spinningTime = timer.milliseconds();
            sorter.setPower(-1);
            Color temp = drum[0];
            drum[0] = drum[1];
            drum[1] = drum[2];
            drum[2] = temp;
        }
    }
    private void sorting() {
        if (spinningTime + 500 < timer.milliseconds()) {
            sorter.setPower(0);
            spinning = false;
            drum[0] = readColor();
        }
    }
    public void moveSorterReverse() {
        if (!spinning) {
            spinning = true;
            spinningTime = timer.milliseconds();
            sorter.setPower(1);
            Color temp = drum[0];
            drum[0] = drum[2];
            drum[2] = drum[1];
            drum[1] = temp;
        }

    }
    public void launchAndSort() {
        if (!flippingAndSpinning) {
            launch();
            flippingAndSpinning = true;
        }
    }
    private void launchingAndSorting() throws InterruptedException {
        synchronized (lock) {
            if (!spinning && flipperTime + 1000 < timer.milliseconds()) {
                moveSorter();
            } else if (spinning && spinningTime + 500 < timer.milliseconds()) {
                flippingAndSpinning = false;
                notify();
            }
        }
    }
    public Color readColor() {
        if (sensor.green() < 100 && sensor.blue() < 100) {
            return Color.EMPTY;
        }
        else if (sensor.green() > sensor.blue()) {
            return Color.GREEN;
        }
        else {
            return Color.PURPLE;
        }
    }


}