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
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.RobotFlags;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Claw;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.concurrent.TimeUnit;

//@Photon
@TeleOp(group = "competition")
public class TeleOpSafe extends LinearOpMode {

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your own robot class
        Robot robot = new Robot(hardwareMap,false);

        double x;
        double y;
        double rx;

//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(4);

        ElapsedTime matchTimer;

        int buzzers = 0;
        double loopTime = 0.0;
        long sumLoop = 0;
        long loopIterations = 0;

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
        boolean previousGP2DUp = false;

        boolean[] detectedIndex;

        boolean smartIntakeEnabled = true;


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;
        matchTimer = new ElapsedTime();
        //intakePreviousPos = robot.intake.intakeMotor.getCurrentPosition();
        robot.slides.runToPosition(0);
        robot.slides.resetAllEncoders();
        robot.intake.resetAllEncoders();
        robot.drone.prime();

        while (opModeIsActive() && !isStopRequested()) {

            x = -gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;

            robot.setDrivePower(-x, y, rx);

            //CLAW
            if (gamepad1.triangle && !previousTriangle) {
                robot.smartClawOrderedOpen();
            }

            if (gamepad1.cross && !previousCross) {
                robot.smartClawOpen();
            }

            if (gamepad1.square && !previousSquare) {
                robot.claw.wrist.setAngle(69);
            }
            if (gamepad1.circle && !previousCircle) {
                robot.claw.wrist.setAngle(169);
            }
//            if (gamepad1.right_trigger > 0.75) {
//                robot.claw.wrist.addAngle(180);
//            }
//            if (gamepad1.circle && !previousCircle) {
//                robot.claw.wrist.setAngle(176);
//            }
            previousCircle = gamepad1.circle;
            previousTriangle = gamepad1.triangle;
            previousSquare = gamepad1.square;
            previousCross = gamepad1.cross;


            //INTAKE
            if (gamepad1.right_bumper && (gamepad1.right_bumper != previousIntakeState)){
                if (robot.intaking) {
                    robot.stopIntake();
                } else {
                    robot.startIntake();
                }
            }
            previousIntakeState = gamepad1.right_bumper;

            if ((gamepad1.left_trigger > 0.2)){
                robot.intake.reverseIntake();
            } else if ((gamepad1.left_trigger < 0.2)  && (gamepad1.left_trigger != previousLeftTriggerState)){
                robot.intake.stopIntake();
            }
            previousLeftTriggerState = gamepad1.left_trigger;

            if ((gamepad1.right_trigger > 0.2) && previousRightTriggerState < 0.2){
//                robot.intake.autoStartIntake(); ;
                if (robot.intake.getAngle() == 215) {
                    robot.intake.setAngle(215 - 17);
                } else if (robot.intake.getAngle() == 198) {
                    robot.intake.setAngle(198 - 17);
                } else if (robot.intake.getAngle() == 181) {
                    robot.intake.setAngle(176);
                } else {
                    robot.intake.setAngle(215);
                }
            }

            previousRightTriggerState = gamepad1.right_trigger;

            //DEPOSIT
            if (gamepad1.left_bumper) {
                robot.depositPreset();
            }

            //SLIDES
            if (gamepad1.dpad_left && !previousDpadLeftState) {
                robot.slides.incrementBackdropTarget(-90);
            } else if (gamepad1.dpad_right && !previousDpadRightState) {
                robot.slides.incrementBackdropTarget(90);
            }
            previousDpadLeftState = gamepad1.dpad_left;
            previousDpadRightState = gamepad1.dpad_right;


            //DRONE
            if (gamepad2.triangle && !previousDroneState) {
                robot.drone.launch();
            }
            previousDroneState = gamepad1.triangle;

            // CLIMB
            if (gamepad1.dpad_up && !previousDpadUp) {
                robot.climbExtend();
            }
            previousDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down) {
                robot.startClimb();
            }

            if (gamepad2.dpad_down) {
                robot.slides.resetAllEncoders();
            }

            if (gamepad2.dpad_up && !previousGP2DUp) {
                smartIntakeEnabled = !smartIntakeEnabled;
                gamepad2.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                        .addStep(1, 1, 250)
                        .addStep(0,0,100)
                        .addStep(1,1,200)
                        .build()
                );
            }
            previousGP2DUp = gamepad2.dpad_up;

//            TIME ALERTS
            if (buzzers == 0 && matchTimer.time(TimeUnit.SECONDS) >= 75) {
                gamepad1.rumble(500);
                buzzers = 1;
            } else if (buzzers == 1 && matchTimer.time(TimeUnit.SECONDS) >= 90) {
                gamepad1.rumble(800);
                gamepad2.rumble(800);
                buzzers = 2;
            }

            robot.slides.update();
            robot.antiJam();
            double loop = System.nanoTime();

//            if (smartIntakeEnabled) {
//                detectedIndex = robot.intakeSensor.hasPixel();
//                if (detectedIndex[0] && detectedIndex[1] && robot.intaking) {
//                    gamepad1.rumble(1, 1, 250);
//                    robot.stopIntake();
//                } else if (detectedIndex[1]) {
//                    gamepad1.rumble(1, 0, 250);
//                } else if (detectedIndex[0]) {
//                    gamepad1.rumble(0, 1, 250);
//                }
//            }

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            sumLoop += 1000000000 / (loop - loopTime);
            loopIterations += 1;
            telemetry.addData("avg hz", (sumLoop / loopIterations));

//            telemetry.addData("SENSOR1 ", robot.intakeSensor.getRangeSensor1());
//            telemetry.addData("SENSOR2 ", robot.intakeSensor.getRangeSensor2());
            telemetry.addData("TIME LEFT: ", ((120-matchTimer.time(TimeUnit.SECONDS))));
            telemetry.addData("SLIDES TARGET: ", robot.slides.target);
            telemetry.addData("INTAKE: ", robot.intake.intakeMotor.motor.getCurrentPosition());
            telemetry.addData("SLIDES POSITION: ", robot.slides.getPos());
            telemetry.addData("LEVEL: ", robot.slides.currentLevel);
            loopTime = loop;
            telemetry.update();
        }
    }
}