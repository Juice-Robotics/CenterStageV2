package org.firstinspires.ftc.teamcode.auton.deprecated;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.YoinkP2Pipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

@Config
@Disabled
@Autonomous(group = "drive")

public class RedSpikeFar extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(62, -34, Math.toRadians(0));
        robot.initPos();
        robot.cv.initProp(AllianceColor.RED);

        drive.setPoseEstimate(startPose);

        // PRELOAD PATHS
        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addTemporalMarker(0, () -> {
                    this.robot.farPos();
                })
                .splineTo(new Vector2d(38, -42), Math.toRadians(180))
                .forward(20)
                .turn(Math.toRadians(-90))
                .strafeLeft(2)
                .build();

        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(58, 20, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(26, 49, Math.toRadians(-90)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(2.6, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(3.6, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(2)
                .strafeLeft(31)
                .back(10)
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addTemporalMarker(0, () -> {
                    this.robot.farPos();
                })
                .back(29)
                .forward(25.5)
                .turn(Math.toRadians(-90))
                .strafeLeft(2)
                .build();

        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(59, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(32, 49), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(2.6, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(3.6, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(2)
                .strafeLeft(26)
                .back(10)
                .build();

        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addTemporalMarker(0, () -> {
                    this.robot.farPos();
                })
                .splineTo(new Vector2d(38, -27), Math.toRadians(135))
                .forward(17)
                .turn(Math.toRadians(-45))
                .build();

        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .strafeLeft(10)
                .back(20)
                .splineToConstantHeading(new Vector2d(59, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(38, 49), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(3, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(4, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(2)
                .strafeLeft(20)
                .back(10)
                .build();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
//        telemetry.addData("Camera State", visionPortal.getCameraState());
//        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
//            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Recorded Position", robot.cv.colourMassDetectionProcessor.getRecordedPropPosition());
            telemetry.addData("Camera State", robot.cv.visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + robot.cv.colourMassDetectionProcessor.getLargestContourX() + ", y: " + robot.cv.colourMassDetectionProcessor.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", robot.cv.colourMassDetectionProcessor.getLargestContourArea());

            telemetry.update();
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */



        waitForStart();

        if (isStopRequested()) return;

        // shuts down the camera once the match starts, we dont need to look any more
        robot.cv.kill();


        // gets the recorded prop position
        YoinkP2Pipeline.PropPositions recordedPropPosition = robot.cv.colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == YoinkP2Pipeline.PropPositions.UNFOUND) {
            recordedPropPosition = YoinkP2Pipeline.PropPositions.CENTER;
        }

        robot.launchSubsystemThread(telemetry);
        switch (recordedPropPosition) {
            case CENTER:
                drive.followTrajectorySequence(preloadSpikeCenter);
                drive.followTrajectorySequence(preloadBackdropCenter);
                break;
            case LEFT:
                drive.followTrajectorySequence(preloadSpikeLeft);
//                drive.followTrajectorySequence(wait);
                drive.followTrajectorySequence(preloadBackdropLeft);
//                drive.followTrajectorySequence(parkLeft);
                break;
            case RIGHT:
                drive.followTrajectorySequence(preloadSpikeRight);
//                drive.followTrajectorySequence(wait);
                drive.followTrajectorySequence(preloadBackdropRight);
//                drive.followTrajectorySequence(parkRight);
                break;
        }

//        drive.followTrajectorySequence(park);



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

        robot.destroyThreads(telemetry);
        robot.cv.kill();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}