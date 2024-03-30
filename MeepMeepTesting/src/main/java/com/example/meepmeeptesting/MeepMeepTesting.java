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
        RoadRunnerBotEntity myBotCycleSafeRed = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)
                .setDimensions(11, 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(62, 13, Math.toRadians(180)))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(35.5, 13, Math.toRadians(180)), Math.toRadians(180))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(34.5, 48, Math.toRadians(-90)))
                        .addTemporalMarker(0, () -> {
                        })
                        .addTemporalMarker(1, () -> {
                        })
                        .addTemporalMarker(2.8, () -> {
                        })
                        .setReversed(false)
                        .addTemporalMarker(1, () -> {
                        })
                        .addTemporalMarker(2.45, () -> {
                        })
                        .splineToConstantHeading(new Vector2d(35.77, -57.7), Math.toRadians(-90))
                        .waitSeconds(0.2)
                        .setReversed(true)
                        .addTemporalMarker(0.5, () -> {
                        })
                        .addTemporalMarker(1.2, () -> {
                        })
                        .addTemporalMarker(2.47, () -> {
                        })
                        .splineToConstantHeading(new Vector2d(36, 47.4), Math.toRadians(90))
                        .waitSeconds(0.1)
                        .setReversed(false)
                        .addTemporalMarker(1, () -> {
                        })
                        .addTemporalMarker(2.4, () -> {
                        })
                        .splineToConstantHeading(new Vector2d(36.2, -57.9), Math.toRadians(-90))
                        .waitSeconds(0.2)
                        .setReversed(true)
                        .addTemporalMarker(0.5, () -> {
                        })
                        .addTemporalMarker(1.2, () -> {
                        })
                        .addTemporalMarker(2.4, () -> {
                        })
                        .splineToConstantHeading(new Vector2d(36, 47), Math.toRadians(90))
                        .waitSeconds(0.1)
                        .setReversed(false)
                        .addTemporalMarker(1, () -> {
                        })
                        .addTemporalMarker(3, () -> {
                        })
                        .splineToConstantHeading(new Vector2d(36, -27), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(25.5, -58.7), Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .setReversed(true)
                        .addTemporalMarker(0.5, () -> {
                        })
                        .addTemporalMarker(1.4, () -> {
                        })
                        .addTemporalMarker(2.6, () -> {
                        })
                        .splineToConstantHeading(new Vector2d(36, -27), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(36, 43), Math.toRadians(90))
                        .waitSeconds(0.1)
                        .setReversed(false)
                        .addTemporalMarker(1, () -> {
                        })
                        .addTemporalMarker(3, () -> {
                        })
                        .splineToConstantHeading(new Vector2d(36, -27), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(25.7, -58.8), Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .setReversed(true)
                        .addTemporalMarker(0.5, () -> {
                        })
                        .addTemporalMarker(1.4, () -> {
                        })
                        .addTemporalMarker(2.6, () -> {
                        })
                        .splineToConstantHeading(new Vector2d(36, -27), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(36, 43), Math.toRadians(90))
                        .waitSeconds(0.1)
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