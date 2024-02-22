package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.relocalization.AprilTagsRelocalization;
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.PreloadPipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.YoinkP2Pipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.YoinkP2Pipeline;
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.PreloadPipeline;
import org.opencv.core.Scalar;

public class CVMaster {
    public VisionPortal visionPortal;
    public YoinkP2Pipeline colourMassDetectionProcessor;
    public PreloadPipeline preloadPipeline;
    AprilTagProcessor processor;
    AllianceColor allianceColor;
    HardwareMap hardwareMap;
    AprilTagProcessor tagProcessor;
    AprilTagsRelocalization relocalization;
    public PreloadPipeline preloadProcessor;

    public CVMaster(HardwareMap map) {
        hardwareMap = map;
    }

    public void initProp(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
        Scalar lower = new Scalar(103, 120, 50);
        Scalar upper = new Scalar(130, 255, 250);

        if (allianceColor == AllianceColor.BLUE) {
            lower = new Scalar(103, 120, 50); // the lower hsv threshold for your detection
            upper = new Scalar(130, 255, 250); // the upper hsv threshold for your detection
        } else if (allianceColor == AllianceColor.RED) {
            lower = new Scalar(125, 120, 50); // the lower hsv threshold for your detection
            upper = new Scalar(190, 255, 250); // the upper hsv threshold for your detection
        }
        double minArea = 3000; // the minimum area for the detection to consider for your prop

        colourMassDetectionProcessor = new YoinkP2Pipeline(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame
        );

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(colourMassDetectionProcessor)
//                .addProcessor(tagProcessor)
                .build();

        visionPortal.setProcessorEnabled(colourMassDetectionProcessor, true);
    }

    public void switchToAutoPipelines() {
        kill();
        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        preloadPipeline = new PreloadPipeline(tagProcessor, allianceColor);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .addProcessors(tagProcessor, preloadPipeline)
//                .addProcessor(tagProcessor)
                .build();
        visionPortal.setProcessorEnabled(tagProcessor, true);
        visionPortal.setProcessorEnabled(preloadPipeline, true);
        FtcDashboard.getInstance().startCameraStream(preloadPipeline, 30);
        relocalization = new AprilTagsRelocalization(tagProcessor);
    }

    public void switchToAprilTags() {
        kill();
        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .addProcessors(tagProcessor)
//                .addProcessor(tagProcessor)
                .build();
        visionPortal.setProcessorEnabled(tagProcessor, true);
        relocalization = new AprilTagsRelocalization(tagProcessor);
    }

    public void switchToAuton(AllianceColor allianceColor) {
        kill();
        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        preloadProcessor = new PreloadPipeline(tagProcessor, allianceColor);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .addProcessors(tagProcessor, preloadProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
//                .addProcessor(tagProcessor)
                .build();
        visionPortal.setProcessorEnabled(tagProcessor, true);
        visionPortal.setProcessorEnabled(preloadProcessor, true);
        relocalization = new AprilTagsRelocalization(tagProcessor);
        startStreamingDashboard();
    }

    public void initPreload(AllianceColor allianceColor) {
        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        preloadProcessor = new PreloadPipeline(tagProcessor, allianceColor);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .addProcessors(tagProcessor, preloadProcessor)
//                .addProcessor(tagProcessor)
                .build();
        visionPortal.setProcessorEnabled(tagProcessor, true);
        visionPortal.setProcessorEnabled(preloadProcessor, true);
        relocalization = new AprilTagsRelocalization(tagProcessor);
        startStreamingDashboard();
    }

    public void initTags() {
        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .addProcessors(tagProcessor)
//                .addProcessor(tagProcessor)
                .build();
        visionPortal.setProcessorEnabled(tagProcessor, true);
        relocalization = new AprilTagsRelocalization(tagProcessor);
    }

    public Pose2d relocalizeUsingBackdrop(Pose2d currentPose) {
        relocalization.detectBackdrop();
        return relocalization.getAbsolutePose2d(currentPose);
    }

    public YoinkP2Pipeline.PropPositions detectPreload() {
        return preloadProcessor.getPreloadedZone();
    }

    public void kill() {
        colourMassDetectionProcessor.close();
        visionPortal.close();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }
    }

    public void startStreamingDashboard() {
        FtcDashboard.getInstance().startCameraStream(preloadProcessor, 0);
    }
}
