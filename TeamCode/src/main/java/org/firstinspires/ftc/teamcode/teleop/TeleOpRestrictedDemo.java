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
public class TeleOpRestrictedDemo extends LinearOpMode {

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your own robot class
        Robot robot = new Robot(hardwareMap,false);

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
        boolean previousGP2DUp = false;

        boolean[] detectedIndex;

        boolean smartIntakeEnabled = true;


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;
        //intakePreviousPos = robot.intake.intakeMotor.getCurrentPosition();
        robot.slides.runToPosition(0);
        robot.slides.resetAllEncoders();
        robot.intake.resetAllEncoders();
        robot.drone.prime();

        while (opModeIsActive() && !isStopRequested()) {

            x = -gamepad1.left_stick_x*0.33;
            y = -gamepad1.left_stick_y*0.33;
            rx = gamepad1.right_stick_x*0.33;

            robot.setDrivePower(-x, y, rx);

            //CLAW
            if (gamepad1.triangle && !previousTriangle) {
                robot.claw.wrist.setAngle(196);
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

            //DEPOSIT
            if (gamepad1.left_bumper) {
                robot.depositPreset();
            }

            //SLIDES
            if (gamepad1.dpad_up && !previousDpadLeftState) {
                robot.slides.incrementBackdropTarget(-130);
            } else if (gamepad1.dpad_down && !previousDpadRightState) {
                robot.slides.incrementBackdropTarget(130);
            }
            previousDpadLeftState = gamepad1.dpad_up;
            previousDpadRightState = gamepad1.dpad_down;


            //DRONE
            if (gamepad2.triangle && !previousDroneState) {
                robot.drone.launch();
            }
            previousDroneState = gamepad1.triangle;

            // CLIMB
//            if (gamepad1.dpad_left && !previousDpadUp) {
//                robot.climbExtend();
//            }
//            previousDpadUp = gamepad1.dpad_left;
//
//            if (gamepad1.dpad_right) {
//                robot.startClimb();
//            }

            if (gamepad2.dpad_down) {
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


            telemetry.addData("SLIDES TARGET: ", robot.slides.target);
            telemetry.addData("INTAKE: ", robot.intake.intakeMotor.motor.getCurrentPosition());
            telemetry.addData("SLIDES POSITION: ", robot.slides.getPos());
            telemetry.addData("LEVEL: ", robot.slides.currentLevel);
            telemetry.update();
        }
    }
}