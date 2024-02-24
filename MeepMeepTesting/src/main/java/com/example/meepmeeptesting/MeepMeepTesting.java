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
                        .splineToLinearHeading(new Pose2d(-34, -32, Math.toRadians(235)), Math.toRadians(30))
                        .forward(12)
                        .turn(Math.toRadians(35))

    //    TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                        .strafeRight(6)
                        //.splineToLinearHeading(new Pose2d(-48, -40, Math.toRadians(235)), Math.toRadians(30))
                        //.splineToLinearHeading(new Pose2d(-57, -40, Math.toRadians(-90)), Math.toRadians(-90))
                        //.setReversed(false)
                        //.splineToConstantHeading(new Vector2d(-57, -25), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-57, 10), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-39, 50), Math.toRadians(90))
                        .addTemporalMarker(0, () -> {
                           // this.robot.intake.setAngle(120);
                        })
                        .addTemporalMarker(2, () -> {
                           // robot.autoPreloadDepositPreset();
                        })
                        .addTemporalMarker(3.5, () -> {
                           // robot.smartClawOpen();
                        })
                        .waitSeconds(3)

                                .build()

                );


        RoadRunnerBotEntity myBotCycleSafeBlue = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-62, -34, Math.toRadians(180)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-39, -47, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(1)

                        .addTemporalMarker(1.8, () -> {
                            //release pixel from intake
                        })
                        //stack
                        //.lineToLinearHeading(new Pose2d(-36, -57, Math.toRadians(-90)))
//                        .lineTo(new Vector2d(-36, -57))
//
//                        .turn(-Math.PI/2)

                       // .setReversed(false)
                        //.splineToConstantHeading(new Vector2d(-36, -57), Math.toRadians(-90))
                                .back(5)
                        .splineToLinearHeading(new Pose2d(-39, -57, Math.toRadians(-90)), Math.toRadians(-90))
                                .strafeLeft(3)
                        .waitSeconds(1.5)
                        .addTemporalMarker(3.7, () -> {
                            //intake
                        })
                        .addTemporalMarker(5.2, () -> {
                            //robot.stopIntake();
                        })
                        .setReversed(true)
                         .strafeRight(0.01)
                        .splineToConstantHeading(new Vector2d(-58, -25), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-58, 15), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-30, 47), Math.toRadians(90))
                        .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(-28, 48), Math.toRadians(90))
                                .build()
                );

        Image img = null;
       // try { img = ImageIO.read(new File("/Users/huntert/Downloads/Juice-CENTERSTAGE-Dark.png")); }
        try { img = ImageIO.read(new File("/Users/zhimi/Downloads/field.png")); }
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