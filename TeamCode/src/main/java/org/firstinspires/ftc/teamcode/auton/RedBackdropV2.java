package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(group = "drive")

public class RedBackdropV2 extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(62, 13, Math.toRadians(0));
        robot.cv.initProp(AllianceColor.RED);
        robot.initPos();

        drive.setPoseEstimate(startPose);

        // PRELOAD PATHS
        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(43, 20, Math.toRadians(270)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(30, 16, Math.toRadians(270)), Math.toRadians(90))
                .addTemporalMarker(1.4, () -> {
                    robot.ejectSpike();
                })
                .waitSeconds(1.3)
                .build();

        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(30, 50.5, Math.toRadians(-90)), Math.toRadians(90))
                .addTemporalMarker(1, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(1.8)
                .forward(4)
                .strafeLeft(29)
                .back(10)
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(22, 33, Math.toRadians(-90)), Math.toRadians(-90))
                .addTemporalMarker(1.7, () -> {
                    robot.ejectSpike();
                })
                .waitSeconds(1.3)
                .build();

        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(37.5, 50.5, Math.toRadians(-90)), Math.toRadians(90))
                .addTemporalMarker(1, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(1.8)
                .forward(4)
                .strafeLeft(23)
                .back(10)
                .build();

        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(22, 33, Math.toRadians(-90)), Math.toRadians(-90))
                .addTemporalMarker(1.7, () -> {
                    robot.ejectSpike();
                })
                .waitSeconds(1.3)
                .build();

        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(37.5, 50.5, Math.toRadians(-90)), Math.toRadians(90))
                .addTemporalMarker(1, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(1.8)
                .forward(4)
                .strafeLeft(23)
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
        robot.cv.switchToAuton(AllianceColor.BLUE);


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
            case RIGHT:
                drive.followTrajectorySequence(preloadSpikeRight);
                drive.followTrajectorySequence(preloadBackdropRight);
                break;
            case LEFT:
                drive.followTrajectorySequence(preloadSpikeLeft);
                drive.followTrajectorySequence(preloadBackdropLeft);
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