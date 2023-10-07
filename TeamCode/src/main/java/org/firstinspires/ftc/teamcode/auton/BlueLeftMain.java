package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.vision.TeamElementCVProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;

@Config
@Autonomous(group = "drive")

public class BlueLeftMain extends LinearOpMode {
    Robot robot;
    VisionPortal visionPortal;
    TeamElementCVProcessor teamElementProcessor;
    TeamElementCVProcessor.Location propLocation = TeamElementCVProcessor.Location.UNFOUND;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(in(92), in(165), rad(90));
        drive.setPoseEstimate(startPose);

        teamElementProcessor = new TeamElementCVProcessor(
                () -> 100, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426, // the left dividing line, in this case the right third of the frame,
                telemetry,
                AllianceColor.BLUE);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(teamElementProcessor)
                .build();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            TeamElementCVProcessor.Location reading = teamElementProcessor.getLocation();
            telemetry.addData("Camera State", visionPortal.getCameraState());

            if (reading == TeamElementCVProcessor.Location.UNFOUND) {
                telemetry.addLine("Team Element Location: <b>NOT FOUND</b>");
            } else {
                telemetry.addData("Team Element Location", reading);
            }

            telemetry.update();
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        // shuts down the camera once the match starts, we dont need to look any more
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        propLocation = teamElementProcessor.getLocation();

        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (propLocation == TeamElementCVProcessor.Location.UNFOUND) {
            propLocation = TeamElementCVProcessor.Location.CENTER;
        }

        waitForStart();

        if (isStopRequested()) return;
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}
