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
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Claw;
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.YoinkP2Pipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

@Config
@Autonomous(group = "drive")

public class RedBackdropSide2Plus8 extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(62, 13, Math.toRadians(180));
        robot.initPos();

        drive.setPoseEstimate(startPose);

        // PRELOAD PATHS
        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(35.5, 13, Math.toRadians(180)), Math.toRadians(180))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(34.5, 48, Math.toRadians(-90)))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.runToPreset(Levels.INTERMEDIATE);
                })
                .addTemporalMarker(1, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2.8, () -> {
                    robot.smartClawOpen();
                })
                .build();

        TrajectorySequence backdropToStacksC1 = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(false)
                .addTemporalMarker(1, () -> {
                    this.robot.startAutoIntake();
                })
                .addTemporalMarker(2.45, () -> {
                    this.robot.intake.setAngle(197);
                })
                .splineToConstantHeading(new Vector2d(35.75, -57.7), Math.toRadians(-90))
                .waitSeconds(0.2)
                .build();

        TrajectorySequence stackToBackdropC1 = drive.trajectorySequenceBuilder(backdropToStacksC1.end())
                .setReversed(true)
                .addTemporalMarker(0.5, () -> {
                    this.robot.stopIntake();
                })
                .addTemporalMarker(1.2, () -> {
                    this.robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(2.45, () -> {
                    robot.smartClawOpen();
                })
                .splineToConstantHeading(new Vector2d(36, 47.4), Math.toRadians(90))
                .waitSeconds(0.1)
                .build();

        TrajectorySequence backdropToStackC2 = drive.trajectorySequenceBuilder(stackToBackdropC1.end())
                .setReversed(false)
                .addTemporalMarker(1, () -> {
                    this.robot.startAutoIntake();
                    this.robot.intake.setAngle(198);
                })
                .addTemporalMarker(2.4, () -> {
                    this.robot.intake.setAngle(177);
                })
                .splineToConstantHeading(new Vector2d(36.15, -58), Math.toRadians(-90))
                .waitSeconds(0.2)
                .build();

        TrajectorySequence stackToBackdropC2 = drive.trajectorySequenceBuilder(backdropToStackC2.end())
                .setReversed(true)
                .addTemporalMarker(0.5, () -> {
                    this.robot.stopIntake();
                })
                .addTemporalMarker(1.2, () -> {
                    this.robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(2.4, () -> {
                    robot.smartClawOpen();
                })
                .splineToConstantHeading(new Vector2d(36, 47), Math.toRadians(90))
                .waitSeconds(0.1)
                .build();

//        TrajectorySequence backdropToStackC3 = drive.trajectorySequenceBuilder(stackToBackdropC2.end())
//                .setReversed(false)
//                .addTemporalMarker(1, () -> {
//                    this.robot.startAutoIntake();
//                })
//                .addTemporalMarker(2.5, () -> {
//                    this.robot.intake.setAngle(198);
//                })
//                .splineToConstantHeading(new Vector2d(36, -27), Math.toRadians(-90))
//                .splineTo(new Vector2d(29.7, -58), Math.toRadians(-130))
//                .waitSeconds(0.5)
//                .build();
//
//        TrajectorySequence stackToBackdropC3 = drive.trajectorySequenceBuilder(backdropToStackC3.end())
//                .setReversed(true)
//                .addTemporalMarker(0.5, () -> {
//                    this.robot.stopIntake();
//                })
//                .splineTo(new Vector2d(34, -27), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(36, 49), Math.toRadians(90))
//                .build();
//
//        TrajectorySequence backdropToStackC4 = drive.trajectorySequenceBuilder(stackToBackdropC3.end())
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(35, -27), Math.toRadians(-90))
//                .splineTo(new Vector2d(27, -60), Math.toRadians(-115))
//                .build();
//
//        TrajectorySequence stackToBackdropC4 = drive.trajectorySequenceBuilder(backdropToStackC4.end())
//                .setReversed(true)
//                .splineTo(new Vector2d(35, -27), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(35, 49), Math.toRadians(90))
//                .build();


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
//        telemetry.addData("Camera State", visionPortal.getCameraState());
//        telemetry.update();

        while (!isStarted() && !isStopRequested())


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */



        waitForStart();

        if (isStopRequested()) return;

        robot.launchSubsystemThread(telemetry);

        drive.followTrajectorySequence(preloadSpikeCenter);
        drive.followTrajectorySequence(backdropToStacksC1);
        drive.followTrajectorySequence(stackToBackdropC1);
        drive.followTrajectorySequence(backdropToStackC2);
        drive.followTrajectorySequence(stackToBackdropC2);
//        drive.followTrajectorySequence(backdropToStackC3);
//        drive.followTrajectorySequence(stackToBackdropC3);
//        drive.followTrajectorySequence(backdropToStackC4);
//        drive.followTrajectorySequence(stackToBackdropC4);



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

        robot.destroyThreads(telemetry);

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}