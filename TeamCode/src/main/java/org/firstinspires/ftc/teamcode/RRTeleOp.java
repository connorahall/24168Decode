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



    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera


    public static Position cameraPosition = new Position(DistanceUnit.INCH,
            1, 3, 8.5, 0);
    public static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -73, 0, 0);

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    SampleMecanumDrive drive;


    // units in inches and seconds
    // launch velocity
    double c = 0.9163542143;
    // gravity
    double a = 386.09;
    // height from launcher to goal
    double h = 40;
    double angle = 53.6;
    ElapsedTime game;
    enum State {
        INTAKE, LAUNCH
    }
    State mode = State.LAUNCH;
    public static int vel;

    Robot bot;

    @Override
    public void runOpMode() throws InterruptedException {



        initAprilTag();


        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx FL = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx FR = hardwareMap.get(DcMotorEx.class, "FR");
        DcMotorEx BL = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx BR = hardwareMap.get(DcMotorEx.class, "BR");
        Servo flipper = hardwareMap.get(Servo.class, "flipper");
        CRServo sorter = hardwareMap.get(CRServo.class, "sorter");
        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "sensor");


        drive = new SampleMecanumDrive(hardwareMap);

        bot = new Robot(drive, gamepad1, launcher, intake, FL, FR, BL, BR, flipper, sorter, sensor, Robot.OpMode.TELEOP);

//        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        bot.getDrive().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));


//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        double launchTablePrecision = 1.;
        HashMap<Integer, Integer> launchTable = new HashMap<>((int) (300 / launchTablePrecision));
        launchTable = generateLaunchTable(launchTablePrecision);

        telemetry.addData("initialization:", "is a success");
        telemetry.update();
        waitForStart();

        bot.setInitialState();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {


            telemetryAprilTag();

            // Push telemetry to the Driver Station.
            telemetry.update();
            drive.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(50);





            System.out.println(distanceFromGoal(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY()));

//            System.out.println(sensor.red() + " " + sensor.green() + " " + sensor.blue());


            telemetry.update();

            bot.setLauncherVelocity(launchTable.get((int)Math.round(distanceFromGoal(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY())))
            - ((int)Math.round(distanceFromGoal(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY())) - 70)*3
            );

            if (gamepad1.dpad_up) {
                mode = State.LAUNCH;
                bot.setLauncherVelocity(vel);
                bot.setIntakePower(-0.5);
            } if (gamepad1.dpad_down) {
                mode = State.INTAKE;
                bot.setLauncherVelocity(0);
                bot.setIntakePower(1);
            } if (gamepad1.dpad_right) {
                bot.setLauncherVelocity(0);
                bot.setIntakePower(0);
            }

            // put a ball into launcher
            if (gamepad1.right_trigger > 0.5) {
                bot.launch();
            }

            // drum sorter spinner code
            if (gamepad1.left_bumper) {
                bot.moveSorter();
            }

            if (gamepad1.right_bumper) {
                bot.moveSorterReverse();
            }

            bot.update();
        }

        visionPortal.close();
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
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
//                    robotPose = to2d(detection.robotPose);
                    drive.setPoseEstimate(new Pose2d(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) + Math.toRadians(90)));
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }



    HashMap<Integer, Integer> generateLaunchTable(double precision) {

        HashMap<Integer, Integer> ret = new HashMap<>((int) (300 / precision));

        // generate table
        // Map distance to the required angle
        double distance;
        for (int i = 100; i < 400; i+=(int)precision) {
            distance = velToDistance(i);
            ret.put((int)Math.round(distance), i);
            System.out.println((int)Math.round(distance) + " " + i);
        }

        return ret;

    }

    double velToDistance(double vel) {


        return ((vel*c) / a) * (Math.cos(angle) * (-1*Math.sin(angle)*(vel*c) - Math.sqrt((vel*c)*(vel*c)*Math.sin(angle)*Math.sin(angle)+(2*a*h))));
    }

//    double ddxVelToDistance(double vel) {
//        return ((vel*c) / a) * (Math.sin(angle) * (Math.sin(angle)*(vel*c) + Math.sqrt((vel*c)*(vel*c)*Math.sin(angle)*Math.sin(angle) + 2*a*h)) - Math.cos(angle)*(Math.cos(angle)*(vel*c)+(2*(vel*c)*(vel*c)*Math.sin(angle)*Math.cos(angle)) / (2*Math.sqrt((vel*c)*(vel*c)*Math.sin(angle)*Math.sin(angle) + 2*a*h))));
//    }

    double distanceFromGoal(double x, double y) {
        // blue goal
        return Math.sqrt((x + 66)*(x+66) + (y+66)*(y+66));
    }

}