package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;
import java.util.Map;

public class Robot {
    SampleMecanumDrive drive;
    Map<String, DcMotorEx> hardwareMotor;
    Map<String, Servo> hardwareServo;
    Map<String, CRServo> hardwareCRServo;
    DcMotorEx launcher, intake, FL, FR, BL, BR;
    Servo flipper;
    CRServo sorter;
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
    OpMode mode = OpMode.AUTO;

    public Robot() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        flipper = hardwareMap.get(Servo.class, "flipper");
        sorter = hardwareMap.get(CRServo.class, "sorter");

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hardwareMotor.put("launcher", launcher);
        hardwareMotor.put("intake", intake);
        hardwareMotor.put("FL", FL);
        hardwareMotor.put("FR", FR);
        hardwareMotor.put("BL", BL);
        hardwareMotor.put("BR", BR);
        hardwareServo.put("flipper", flipper);
        hardwareCRServo.put("sorter", sorter);

        drive = new SampleMecanumDrive(hardwareMap);
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

    public Robot(Pose2d startingPose) {
        this();
        drive.setPoseEstimate(startingPose);
    }

    public void update() throws InterruptedException {

        System.out.println(launcher.getVelocity(AngleUnit.DEGREES));

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

        }
    }

    private void checkhardware(String device) {
        if (hardwareMotor.get(device) == null) {
            System.out.println("Motor '" + device + "' is null");
        } else if (hardwareServo.get(device) == null) {
            System.out.println("Servo '" + device + "' is null");
        } else if (hardwareServo.get(device) == null) {
            System.out.println("CRServo '" + device + "' is null");
        }
    }
    public DcMotorEx getMotor(String motor) {
        checkhardware(motor);
        return hardwareMotor.get(motor);
    }
    public Servo getServo(String servo) {
        checkhardware(servo);
        return hardwareServo.get(servo);
    }
    public CRServo getCRServo(String cRServo) {
        checkhardware(cRServo);
        return hardwareCRServo.get(cRServo);
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
        intake.setPower(0.5);
        sorter.setPower(0);
        flipper.setPosition(0);
    }
    public void setRestState() {
        launcher.setPower(0);
        sorter.setPower(0);
        flipper.setPosition(0);
        intake.setPower(0);
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
        }
    }
    public void moveSorter() {
        if (!spinning) {
            spinning = true;
            spinningTime = timer.milliseconds();
            sorter.setPower(-1);
        }
    }
    private void sorting() {
        if (spinningTime + 500 < timer.milliseconds()) {
            sorter.setPower(0);
            spinning = false;
        }
    }
    public void moveSorterReverse() {

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


}