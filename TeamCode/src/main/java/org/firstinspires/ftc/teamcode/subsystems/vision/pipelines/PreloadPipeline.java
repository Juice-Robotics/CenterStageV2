package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines;

import android.graphics.Bitmap;
import android.graphics.Canvas;

<<<<<<< HEAD
=======
import org.firstinspires.ftc.teamcode.subsystems.vision.pipelines.YoinkP2Pipeline;
>>>>>>> 7968afc6a37f278c43ae444f3a284bfbd0414c23
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
<<<<<<< HEAD
=======
import org.opencv.android.Utils;
>>>>>>> 7968afc6a37f278c43ae444f3a284bfbd0414c23
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class PreloadPipeline implements VisionProcessor, CameraStreamSource {

    private int targetAprilTagID = 0;

    private YoinkP2Pipeline.PropPositions preloadedZone = YoinkP2Pipeline.PropPositions.CENTER;

    private AprilTagProcessor aprilTag;
    private AllianceColor allianceColor = AllianceColor.BLUE;

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public PreloadPipeline(AprilTagProcessor aprilTag, AllianceColor color) {
        this.aprilTag = aprilTag;
        allianceColor = color;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
<<<<<<< HEAD

=======
        lastFrame.set(Bitmap.createBitmap(1120, 630, Bitmap.Config.RGB_565));
>>>>>>> 7968afc6a37f278c43ae444f3a284bfbd0414c23
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == targetAprilTagID) {
                        int leftX = Integer.MAX_VALUE;
                        int rightX = Integer.MIN_VALUE;
                        int topY = Integer.MIN_VALUE;
                        int bottomY = Integer.MAX_VALUE;

                        for (Point point : detection.corners) {
                            if (point.x < leftX) leftX = (int) point.x;
                            if (point.x > rightX) rightX = (int) point.x;
                            if (point.y > topY) topY = (int) point.y;
                            if (point.y < bottomY) bottomY = (int) point.y;
                        }

                        int tagCenterX = (int) detection.center.x;
                        int tagCenterY = (int) detection.center.y;

                        int tagWidth = rightX - leftX;
                        int tagHeight = topY - bottomY;

                        int inclusionZoneWidth = (int) (tagWidth * 1.5);
                        int inclusionZoneHeight = (int) (tagHeight * 1.5);

                        int exclusionZoneWidth = (int) (tagWidth * 0.28);
                        int exclusionZoneHeight = (int) (tagHeight * 0.28);

<<<<<<< HEAD
                        Rect leftInclusionZone = new Rect(tagCenterX - inclusionZoneWidth, tagCenterY - 110, inclusionZoneWidth, inclusionZoneHeight);
                        Rect rightInclusionZone = new Rect(tagCenterX, tagCenterY - 110, inclusionZoneWidth, inclusionZoneHeight);
=======
                        Rect leftInclusionZone = new Rect(tagCenterX - inclusionZoneWidth, tagCenterY + 30, inclusionZoneWidth, inclusionZoneHeight);
                        Rect rightInclusionZone = new Rect(tagCenterX, tagCenterY + 30, inclusionZoneWidth, inclusionZoneHeight);
>>>>>>> 7968afc6a37f278c43ae444f3a284bfbd0414c23

                        Rect leftExclusionZone = new Rect(tagCenterX - (int) (inclusionZoneWidth * 0.64), tagCenterY - 90, exclusionZoneWidth, exclusionZoneHeight);
                        Rect rightExclusionZone = new Rect(tagCenterX + (int) (inclusionZoneWidth * 0.28), tagCenterY - 90, exclusionZoneWidth, exclusionZoneHeight);

                        Imgproc.rectangle(frame, leftInclusionZone, new Scalar(0, 255, 0), 7);
                        Imgproc.rectangle(frame, rightInclusionZone, new Scalar(0, 255, 0), 7);

                        int leftZoneAverage = meanColor(frame, leftInclusionZone, leftExclusionZone);
                        int rightZoneAverage = meanColor(frame, rightInclusionZone, rightExclusionZone);

//                        System.out.println("LEFTAVG " + leftZoneAverage);
//                        System.out.println("RIGHTAVG " + rightZoneAverage);

                        preloadedZone = (leftZoneAverage > rightZoneAverage) ? YoinkP2Pipeline.PropPositions.LEFT : YoinkP2Pipeline.PropPositions.RIGHT;
                        System.out.println("PRELOADED ZONE: " + preloadedZone);
<<<<<<< HEAD
=======


>>>>>>> 7968afc6a37f278c43ae444f3a284bfbd0414c23
//                        Globals.PRELOAD = preloadedZone;
                    }
                }
            }
        }

<<<<<<< HEAD
=======
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);

>>>>>>> 7968afc6a37f278c43ae444f3a284bfbd0414c23

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public YoinkP2Pipeline.PropPositions getPreloadedZone() {
        return this.preloadedZone;
    }

    public int getTargetAprilTagID() {
        return this.targetAprilTagID;
    }

    public void setTargetAprilTagID(YoinkP2Pipeline.PropPositions preloadLocation) {
        targetAprilTagID = 0;
        switch (preloadLocation) {
            case LEFT:
                targetAprilTagID = 1;
                break;
            case CENTER:
                targetAprilTagID = 2;
                break;
            case RIGHT:
                targetAprilTagID = 3;
                break;
            default:
                break;
        }

        if (allianceColor == AllianceColor.RED) targetAprilTagID += 3;
    }

    public int meanColor(Mat frame, Rect inclusionRect, Rect exclusionRect) {
        if (frame == null) {
            System.out.println("frame is bad");
            return 0;
        }

        int sum = 0;
        int count = 0;
        for (int y = inclusionRect.y; y < inclusionRect.y + inclusionRect.height; y++) {
            for (int x = inclusionRect.x; x < inclusionRect.x + inclusionRect.width; x++) {
                if (x < 0 || x >= frame.cols() || y < 0 || y >= frame.rows()) {
                    continue;
                }

                if (x >= exclusionRect.x && x < exclusionRect.x + exclusionRect.width && y >= exclusionRect.y && y < exclusionRect.y + exclusionRect.height) {
                    continue;
                }

                double[] data = frame.get(y, x);
                if (data != null && data.length > 0) {
                    sum += data[0];
                    count++;
                }
            }
        }

        return count > 0 ? sum / count : 0;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

}