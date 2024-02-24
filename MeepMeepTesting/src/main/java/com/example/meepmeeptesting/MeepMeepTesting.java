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
        RoadRunnerBotEntity myBotCycleSafeBlue = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-62, -34, Math.toRadians(180)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-32, -34, Math.toRadians(90)), Math.toRadians(90))
                        .addTemporalMarker(1.8, () -> {
                            //release pixel from intake
                        })
                        .waitSeconds(1)
                        //stack
                        .splineToLinearHeading(new Pose2d(-12, -57, Math.toRadians(-90)), Math.toRadians(-90))
                        .waitSeconds(1.5)

                        .addTemporalMarker(0.7, () -> {
                            //intake
                        })
                        .addTemporalMarker(2.1, () -> {
                            //robot.stopIntake();
                        })
                        .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("/Users/huntert/Downloads/Juice-CENTERSTAGE-Dark.png")); }
//        try { img = ImageIO.read(new File("/Users/zhimi/Downloads/field.png")); }
        //try { img = ImageIO.read(new File("/Users/siddharth/dev/Juice/CenterStage/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/Juice-CENTERSTAGE-Dark.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBotCycleSafeBlue)
                .start();
    }
}