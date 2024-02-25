package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);
        RoadRunnerBotEntity myBotCycleSafeRedRight = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(62, 13, Math.toRadians(0)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(22, 33, Math.toRadians(-90)), Math.toRadians(-90))
                        .addTemporalMarker(1.7, () -> {
//                            robot.ejectSpike();
                        })
                        .waitSeconds(1.3)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(37.5, 50.5, Math.toRadians(-90)), Math.toRadians(90))
                        .addTemporalMarker(1, () -> {
//                            robot.autoPreloadDepositPreset();
                        })
                        .addTemporalMarker(2, () -> {
//                            robot.smartClawOpen();
                        })
                        .waitSeconds(1.8)
                        .forward(4)
                        .strafeLeft(23)
                        .back(10)
                        .build()
                );

        RoadRunnerBotEntity myBotCycleSafeRedCenter = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(62, 13, Math.toRadians(0)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(22, 33, Math.toRadians(-90)), Math.toRadians(-90))
                        .addTemporalMarker(1.7, () -> {
//                            robot.ejectSpike();
                        })
                        .waitSeconds(1.3)

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(37.5, 50.5, Math.toRadians(-90)), Math.toRadians(90))
                        .addTemporalMarker(1, () -> {
//                            robot.autoPreloadDepositPreset();
                        })
                        .addTemporalMarker(2, () -> {
//                            robot.smartClawOpen();
                        })
                        .waitSeconds(1.8)
                        .forward(4)
                        .strafeLeft(23)
                        .back(10)
                                .build()
                );

        RoadRunnerBotEntity myBotCycleSafeRedLeft = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(62, 13, Math.toRadians(0)))
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(43, 20, Math.toRadians(270)), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(30, 16, Math.toRadians(270)), Math.toRadians(90))
                        .addTemporalMarker(1.4, () -> {
//                            robot.ejectSpike();
                        })
                        .waitSeconds(1.3)


                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(30, 50.5, Math.toRadians(-90)), Math.toRadians(90))
                        .addTemporalMarker(1, () -> {
//                            robot.autoPreloadDepositPreset();
                        })
                        .addTemporalMarker(2, () -> {
//                            robot.smartClawOpen();
                        })
                        .waitSeconds(1.8)
                        .forward(4)
                        .strafeLeft(29)
                        .back(10)
                                .build()
                );

        RoadRunnerBotEntity myBotCycleSafeRed = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(62, 13, Math.toRadians(0)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(31, 42, Math.toRadians(-90)), Math.toRadians(90))
                                .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("/Users/huntert/Downloads/Juice-CENTERSTAGE-Dark.png")); }
//        try { img = ImageIO.read(new File("/Users/zhimi/Downloads/field.png")); }
//        try { img = ImageIO.read(new File("/Users/siddharth/dev/Juice/CenterStage/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/Juice-CENTERSTAGE-Dark.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(myBotCycleSafeRedLeft)
                .addEntity(myBotCycleSafeRed)
                .start();
    }
}