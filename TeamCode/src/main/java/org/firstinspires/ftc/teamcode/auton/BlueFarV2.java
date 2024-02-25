package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

public class BlueFarV2 extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(-62, -36, Math.toRadians(180));
        robot.cv.initProp(AllianceColor.BLUE);
        robot.initPos();

        drive.setPoseEstimate(startPose);

        // PRELOAD PATHS
        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-32, -34, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-58, -30, Math.toRadians(-90)), Math.toRadians(90))
                .addTemporalMarker(0.5, () -> {
                    robot.ejectSpike();
                })
                .waitSeconds(2)
//                .splineToLinearHeading(new Pose2d(-33, -55, Math.toRadians(-90)), Math.toRadians(-90))
//                .addTemporalMarker(5, () -> {
//                    robot.startAutoIntake();
//                })
//                .waitSeconds(1)
//                .forward(2.5)
//                .addTemporalMarker(6.5, () -> {
//                    robot.stopIntake();
//                })
//                .waitSeconds(3)
                .build();

        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(true)
                //.splineToConstantHeading(new Vector2d(-58, -28), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-58, 15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-40, 40), Math.toRadians(90))
                .waitSeconds(3)
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-35, -36, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-58, -25, Math.toRadians(-90)), Math.toRadians(90))
                //.splineToLinearHeading(new Pose2d(-48, 13, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(1.8, () -> {
                    //release pixel from intake
                })
//                //stack
//                .splineToLinearHeading(new Pose2d(-36, -57, Math.toRadians(-90)), Math.toRadians(-90))
//                .waitSeconds(1.5)
                .build();

        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                //break
                .setReversed(true)
                //.splineToConstantHeading(new Vector2d(-58, -25), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-58, 15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-36, 47), Math.toRadians(90))
                .waitSeconds(1)
                .build();

        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-32, -36, Math.toRadians(-90)), Math.toRadians(0))
                .waitSeconds(1)

                .addTemporalMarker(1.8, () -> {
                    //release pixel from intake
                })
                //stack
//                .back(5)
//                .splineToLinearHeading(new Pose2d(-39, -57, Math.toRadians(-90)), Math.toRadians(-90))
//                .strafeLeft(3)
//                .waitSeconds(1.5)
//                .addTemporalMarker(3.7, () -> {
//                    //intake
//                })
//                .addTemporalMarker(5.2, () -> {
//                    //robot.stopIntake();
//                })
                .build();


        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .setReversed(true)
                .strafeRight(3)
                .splineToConstantHeading(new Vector2d(-58, -29), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-58, 15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-30, 47), Math.toRadians(90))
                .waitSeconds(1)
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
            recordedPropPosition = YoinkP2Pipeline.PropPositions.LEFT;
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