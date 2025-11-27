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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
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
    double cameraTime = 0;

    int goalX = 66;
    int goalY = 66;
    ElapsedTime timer;
    private final Object lock;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera


    public static Position cameraPosition = new Position(DistanceUnit.INCH,
            1, 3, 8.5, 0);
    public static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -73, 0, 0);


    private AprilTagProcessor aprilTag;


    public VisionPortal visionPortal;

    private HashMap<Integer, Integer> launchTable;

    public enum OpMode {
        AUTO, TELEOP
    }
    public enum Color {
        PURPLE, GREEN, BLUE, RED, EMPTY
    }
    Color[] drum = {Color.EMPTY, Color.EMPTY, Color.EMPTY};
    Color team = Color.RED;

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

        initAprilTag();

        double launchTablePrecision = 1.;
        launchTable = new HashMap<>((int) (500 / launchTablePrecision));
        launchTable = generateLaunchTable(launchTablePrecision);
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

//        telemetryAprilTag();

        // Push telemetry to the Driver Station.
//        telemetry.update();
        drive.update();


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

        cameraTelemetryAtInterval();

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

    private int getAutoPower() {
        int distance = (int) Math.round(distanceFromGoal());
        try {
            return launchTable.get(distance) - ((distance - 72) * 3);
        } catch (Exception e) {
            // if it fails, keep launcher at the same velocity
            System.out.println("launchTable get() error at distance " + distance);
            return (int)launcher.getVelocity(AngleUnit.DEGREES);
        }
    }
    public void autoPowerLauncher() {
        launcher.setVelocity(getAutoPower());
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    public AprilTagDetection telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
//                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
//                            detection.robotPose.getPosition().x,
//                            detection.robotPose.getPosition().y,
//                            detection.robotPose.getPosition().z));
//                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
//                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
//                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
//                    robotPose = to2d(detection.robotPose);
                    drive.setPoseEstimate(new Pose2d(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) + Math.toRadians(90)));
                    return detection;
                }
            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
        return null;
        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }

    public Color identifyMyTeam() {
        AprilTagDetection detection = telemetryAprilTag();
        if (detection != null) {
            // if I can see blue goal, I am red
            if (detection.id == 20) {
                return Color.RED;
            }
            // if I can see red goal, I am blue
            else if (detection.id == 24) {
                return Color.BLUE;
            }
        }
        // default is red
        return Color.RED;
    }

    public void cameraTelemetryAtInterval() {
        // run every second
        if (cameraTime + 1000 < timer.milliseconds()) {
            visionPortal.resumeLiveView();
            cameraTime = timer.milliseconds();
            telemetryAprilTag();
            visionPortal.stopLiveView();
        }
    }

    HashMap<Integer, Integer> generateLaunchTable(double precision) {

        HashMap<Integer, Integer> ret = new HashMap<>((int) (500 / precision));

        // generate table
        // Map distance to the required angle
        double distance;
        for (int i = 0; velToDistance(i) < 250; i+=(int)precision) {
            distance = velToDistance(i);
            ret.put((int)Math.round(distance), i);
            System.out.println((int)Math.round(distance) + " " + i);
        }

        return ret;

    }

    double velToDistance(double vel) {


        // units in inches and seconds
        // gravity
        double a = 386.09;
        // launch velocity
        double c = 0.9163542143;
        // height from launcher to goal
        double h = 40;
        double angle = 53.6;
        return ((vel* c) / a) * (Math.cos(angle) * (-1*Math.sin(angle)*(vel* c) - Math.sqrt((vel* c)*(vel* c)*Math.sin(angle)*Math.sin(angle)+(2* a * h))));
    }

//    double ddxVelToDistance(double vel) {
//        return ((vel*c) / a) * (Math.sin(angle) * (Math.sin(angle)*(vel*c) + Math.sqrt((vel*c)*(vel*c)*Math.sin(angle)*Math.sin(angle) + 2*a*h)) - Math.cos(angle)*(Math.cos(angle)*(vel*c)+(2*(vel*c)*(vel*c)*Math.sin(angle)*Math.cos(angle)) / (2*Math.sqrt((vel*c)*(vel*c)*Math.sin(angle)*Math.sin(angle) + 2*a*h))));
//    }

    private double distanceFromGoal() {
        double x = drive.getPoseEstimate().getX();
        double y = drive.getPoseEstimate().getY();
        x = x > 72 ? 72 : x;
        x = x < -72 ? -72 : x;
        y = y > 72 ? 72 : y;
        y = y < -72 ? -72 : y;
        if (team == Color.RED)
            return Math.sqrt(Math.pow(x+goalX, 2) + Math.pow(y-goalY, 2));
        return Math.sqrt(Math.pow(x+goalX, 2) + Math.pow(y+goalY, 2));
    }
    private double headingDeviationFromGoal() {
        double x = drive.getPoseEstimate().getX();
        double y = drive.getPoseEstimate().getY();
        double heading = drive.getPoseEstimate().getHeading();

        if (team == Color.BLUE) {
            return heading - (Math.atan2(y + goalY, x + goalX) + 180);
        } else {
            return heading - (Math.atan2(y - goalY, x + goalX) + 180);
        }
    }

    public void setTeam(Color team) {
        this.team = team;
    }
    public Color getTeam() {
        return team;
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
    private void launchingAndSorting() {
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