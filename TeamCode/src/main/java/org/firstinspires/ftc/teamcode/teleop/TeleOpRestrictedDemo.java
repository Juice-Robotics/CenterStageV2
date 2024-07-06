package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.RobotFlags;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Claw;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.concurrent.TimeUnit;

//@Photon
@TeleOp(group = "competition")
public class TeleOpRestrictedDemo extends LinearOpMode {

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller

    /**
     * States
     * 0-nuetral
     * 1-intaking
     * 2-deposit pos
     * 3-claw open
     */
    int currentBasicState = 0;

    Vector2d FIELD_DIMENSIONS = new Vector2d(24, 24);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your own robot class
        Robot robot = new Robot(hardwareMap,false);

        Gamepad.LedEffect disabledEffect = new Gamepad.LedEffect.Builder()
                .addStep(1, 0, 0, 250) // Show red for 250ms
                .addStep(0, 0, 0, 250) // Show green for 250ms
                .addStep(1, 0, 0, 250) // Show blue for 250ms
                .addStep(0, 0, 0, 250) // Show white for 250ms
                .addStep(1, 0, 0, 250) // Show red for 250ms
                .addStep(0, 0, 0, 250) // Show green for 250ms
                .addStep(1, 0, 0, 250) // Show blue for 250ms
                .addStep(0, 0, 0, 250) // Show white for 250ms
                .addStep(1, 0, 0, 250) // Show red for 250ms
                .addStep(0, 0, 0, 250) // Show green for 250ms
                .addStep(1, 0, 0, 250) // Show blue for 250ms
                .addStep(0, 0, 0, 250) // Show white for 250ms
                .addStep(1, 0, 0, 250) // Show red for 250ms
                .addStep(0, 0, 0, 250) // Show green for 250ms
                .addStep(1, 0, 0, 250) // Show blue for 250ms
                .addStep(0, 0, 0, 250) // Show white for 250ms
                .build();

        double x;
        double y;
        double rx;

        boolean previousDpadLeftState = false;
        boolean previousDpadRightState = false;
        boolean previousDroneState = false;
        boolean previousIntakeState = false;
        boolean previousDpadUp = false;
        float previousLeftTriggerState = 0;
        float previousRightTriggerState = 0;
        boolean previousSquare = false;
        boolean previousTriangle = false;
        boolean previousCross = false;
        boolean previousCircle = false;
        boolean previousTriangleBasic = false;

        boolean[] detectedIndex;

        boolean smartIntakeEnabled = true;

        boolean disabled = false;


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // This is assuming you're using StandardTrackingWheelLocalizer.java
        // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your configuration is different
        TwoWheelTrackingLocalizer myLocalizer = new TwoWheelTrackingLocalizer(hardwareMap, robot.drive);

        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        myLocalizer.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        waitForStart();
        if (isStopRequested()) return;
        currentBasicState = 0;
        //intakePreviousPos = robot.intake.intakeMotor.getCurrentPosition();
        robot.slides.runToPosition(0);
        robot.slides.resetAllEncoders();
        robot.intake.resetAllEncoders();
        robot.drone.prime();

        while (opModeIsActive() && !isStopRequested()) {
            myLocalizer.update();

            if (!disabled && !inFieldBounds(myLocalizer.getPoseEstimate())) {
                disabled = true;
                gamepad1.runLedEffect(disabledEffect);
                gamepad1.rumble(500);
            }

            if (!disabled) {
                x = -gamepad1.left_stick_x * 0.33;
                y = -gamepad1.left_stick_y * 0.33;
                rx = gamepad1.right_stick_x * 0.33;

                robot.setDrivePower(-x, y, rx);

                if (gamepad1.triangle && !previousTriangleBasic) {
                    switch (currentBasicState) {
                        case 0:
                            robot.startIntake();
                            currentBasicState = 1;
                            break;
                        case 1:
                            if (robot.subsystemState != Levels.INTERMEDIATE) {
                                robot.stopIntakeAndClaw();
                            } else {
                                robot.depositPreset();
                            }
                            currentBasicState = 2;
                            break;
                        case 2:
                            robot.smartClawOpen();
                            currentBasicState = 0;
                            break;
                    }
                }
                previousTriangleBasic = gamepad1.triangle;

            } else {
                x = -gamepad2.left_stick_x * 0.33;
                y = -gamepad2.left_stick_y * 0.33;
                rx = gamepad2.right_stick_x * 0.33;

                robot.setDrivePower(-x, y, rx);

                if (gamepad2.cross && !previousCross) {
                    disabled = false;
                }
            }

            //CLAW
            if (gamepad2.triangle && !previousTriangle) {
                robot.claw.wrist.setAngle(169);
            }

            if (gamepad2.cross && !previousCross && !disabled) {
                robot.smartClawOpen();
            }

            if (gamepad2.square && !previousSquare) {
                robot.claw.wrist.setAngle(69);
            }
//            if (gamepad1.right_trigger > 0.75) {
//                robot.claw.wrist.addAngle(180);
//            }
//            if (gamepad1.circle && !previousCircle) {
//                robot.claw.wrist.setAngle(176);
//            }
            previousCircle = gamepad2.circle;
            previousTriangle = gamepad2.triangle;
            previousSquare = gamepad2.square;
            previousCross = gamepad2.cross;


            //INTAKE
            if (gamepad2.right_bumper && (gamepad2.right_bumper != previousIntakeState)) {
                if (robot.intaking) {
                    robot.stopIntake();
                } else {
                    robot.startIntake();
                }
            }
            previousIntakeState = gamepad2.right_bumper;

            if ((gamepad2.left_trigger > 0.2)) {
                robot.intake.reverseIntake();
            } else if ((gamepad2.left_trigger < 0.2) && (gamepad2.left_trigger != previousLeftTriggerState)) {
                robot.intake.stopIntake();
            }
            previousLeftTriggerState = gamepad2.left_trigger;

            //DEPOSIT
            if (gamepad2.left_bumper) {
                robot.depositPreset();
            }

            //SLIDES
            if (gamepad2.dpad_down && !previousDpadLeftState) {
                robot.slides.incrementBackdropTarget(-130);
            } else if (gamepad2.dpad_up && !previousDpadRightState) {
                robot.slides.incrementBackdropTarget(130);
            }
            previousDpadLeftState = gamepad2.dpad_down;
            previousDpadRightState = gamepad2.dpad_up;


            //DRONE
//            if (gamepad2.triangle && !previousDroneState) {
//                robot.drone.launch();
//            }
//            previousDroneState = gamepad1.triangle;

            if (gamepad2.dpad_left) {
                robot.slides.resetAllEncoders();
            }

            robot.slides.update();
            robot.antiJam();

            if (true) {
                detectedIndex = robot.intakeSensor.hasPixel();
                if (detectedIndex[0] && detectedIndex[1] && robot.intaking) {
                    gamepad1.rumble(1, 1, 250);
                    robot.stopIntake();
                } else if (detectedIndex[1]) {
                    gamepad1.rumble(1, 0, 250);
                } else if (detectedIndex[0]) {
                    gamepad1.rumble(0, 1, 250);
                }
            }


            // Retrieve your pose
            Pose2d myPose = myLocalizer.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());
            telemetry.addData("SLIDES TARGET: ", robot.slides.target);
            telemetry.addData("INTAKE: ", robot.intake.intakeMotor.motor.getCurrentPosition());
            telemetry.addData("SLIDES POSITION: ", robot.slides.getPos());
            telemetry.addData("LEVEL: ", robot.slides.currentLevel);
            telemetry.update();
        }
    }

    public boolean inFieldBounds(Pose2d pose) {
        return pose.getX() <= FIELD_DIMENSIONS.getX() && pose.getY() <= FIELD_DIMENSIONS.getY() && pose.getX() >= -2 && pose.getY() >= -2;
    }
}