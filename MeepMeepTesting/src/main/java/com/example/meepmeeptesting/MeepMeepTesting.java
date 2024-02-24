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


        RoadRunnerBotEntity myBotCyclesSafeRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(15, 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(62, -34, Math.toRadians(0)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(22, -47, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(1)

                        .addTemporalMarker(1.8, () -> {
                            //release pixel from intake
                        })
                        //stack
                        .splineToLinearHeading(new Pose2d(12, -57, Math.toRadians(-90)), Math.toRadians(-90))
                        .waitSeconds(1.5)
                        .addTemporalMarker(3.5, () -> {
                            //this.robot.intake.setAngle(120);
                        })
                        .addTemporalMarker(3.7, () -> {
                            //intake
                        })
                        .addTemporalMarker(5.2, () -> {
                            //robot.stopIntake();
                        })
        //TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                        .splineToConstantHeading(new Vector2d(10, 25), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(42, 47.7), Math.toRadians(90))
                        .waitSeconds(1)
                                .build()

                );

        Image img = null;
        try { img = ImageIO.read(new File("/Users/zhimi/Downloads/field.png")); }
        //try { img = ImageIO.read(new File("/Users/siddharth/dev/Juice/CenterStage/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/Juice-CENTERSTAGE-Dark.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(myBotCyclesUnsafeSide)
                .addEntity(myBotCyclesSafeRed)
//                .addEntity(myBotCycleSafeBlue)
//                .addEntity(myBotCyclesSafeOpti)
                //.addEntity(myBotRelocal)
                .start();
    }
}