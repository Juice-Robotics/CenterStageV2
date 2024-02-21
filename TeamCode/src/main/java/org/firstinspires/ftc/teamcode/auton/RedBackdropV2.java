package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.YoinkP2Pipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;

@Config
@Autonomous(group = "drive")

public class RedBackdropV2 extends LinearOpMode {
    Robot robot;
    CVMaster cv;


    @Override
    public void runOpMode() throws InterruptedException {
        Scalar lower = new Scalar(103, 120, 50); // the lower hsv threshold for your detection
        Scalar upper = new Scalar(130, 255, 250); // the upper hsv threshold for your detection
        double minArea = 3000; // the minimum area for the detection to consider for your prop

        cv = new CVMaster(hardwareMap);
        cv.initProp(AllianceColor.RED);

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(62, 13, Math.toRadians(0));
        robot.initPos();

        drive.setPoseEstimate(startPose);

        // PRELOAD PATHS
        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(32, 32, Math.toRadians(-90)), Math.toRadians(-90))
                .addTemporalMarker(1.8, () -> {
                    //release pixel from intake
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(44, 49, Math.toRadians(-90)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(1.1, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2.1, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(2)
                .strafeLeft(15)
                .back(8)
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(26, 25, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1)
                //.splineToLinearHeading(new Pose2d(-48, 13, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(1.8, () -> {
                    //release pixel from intake
                })
                .build();

        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(35, 49, Math.toRadians(-90)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(1.6, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2.6, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(2)
                .strafeLeft(23)
                .back(8)
                .build();

        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(43, 14, Math.toRadians(-80)), Math.toRadians(170))
                .splineToSplineHeading(new Pose2d(32, 11, Math.toRadians(-90)), Math.toRadians(270))
                .waitSeconds(1)
                .addTemporalMarker(1.8, () -> {
                    //release pixel from intake
                })
                .build();

        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(29, 49, Math.toRadians(-90)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(1.1, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2.1, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(2)
                .strafeLeft(28)
                .back(8)
                .build();


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
//        telemetry.addData("Camera State", visionPortal.getCameraState());
//        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
//            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Recorded Position", cv.colourMassDetectionProcessor.getRecordedPropPosition());
            telemetry.addData("Camera State", cv.visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + cv.colourMassDetectionProcessor.getLargestContourX() + ", y: " + cv.colourMassDetectionProcessor.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", cv.colourMassDetectionProcessor.getLargestContourArea());

            telemetry.update();
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */



        waitForStart();

        if (isStopRequested()) return;


        // gets the recorded prop position
        YoinkP2Pipeline.PropPositions recordedPropPosition = cv.colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == YoinkP2Pipeline.PropPositions.UNFOUND) {
            recordedPropPosition = YoinkP2Pipeline.PropPositions.CENTER;
        }

        recordedPropPosition = YoinkP2Pipeline.PropPositions.RIGHT;
        // shuts down the camera once the match starts, we dont need to look any more
        cv.switchToAuton(AllianceColor.RED);

        robot.launchSubsystemThread(telemetry);
        switch (recordedPropPosition) {
            case CENTER:
                drive.followTrajectorySequence(preloadSpikeCenter);
                drive.followTrajectorySequence(preloadBackdropCenter);
//                drive.followTrajectorySequence(centerCycle1);
//                drive.followTrajectorySequence(centerCycle2);
                break;
            case LEFT:
                drive.followTrajectorySequence(preloadSpikeRight);
                drive.followTrajectorySequence(preloadBackdropRight);
                //drive.followTrajectorySequence(rightCycle1);
                break;
            case RIGHT:
                drive.followTrajectorySequence(preloadSpikeLeft);
                drive.followTrajectorySequence(preloadBackdropLeft);
                //drive.followTrajectorySequence(leftCycle1);
                break;
        }

//        drive.followTrajectorySequence(park);



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

        robot.destroyThreads(telemetry);
        cv.kill();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}