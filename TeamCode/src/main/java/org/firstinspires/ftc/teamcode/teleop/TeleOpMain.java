package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.RobotFlags;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "competition")
public class TeleOpMain extends LinearOpMode {
    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }
    private Mode currentMode = Mode.NORMAL_CONTROL;
    private CVMaster cv;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDriveCancelable.HEADING_PID);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your own robot class
        Robot robot = new Robot(hardwareMap,false);

        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        headingController.setInputBounds(-Math.PI, Math.PI);

        double x;
        double y;
        double rx;

//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(4);
//        PhotonCore.enable();
//        PhotonCore.start(hardwareMap);

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
        boolean previousSquare = false;
        boolean previousCross = false;
        boolean previousCircle = false;
        boolean previousGP2DUp = false;
        boolean previousAutoAlignState = false;
        boolean previousRelocalizeState = false;

        boolean[] detectedIndex;

        boolean smartIntakeEnabled = true;


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;
        matchTimer = new ElapsedTime();
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.setPoseEstimate(PoseStorage.currentPose);
        cv = new CVMaster(hardwareMap);
        cv.initTags();
        Pose2d relocalizePoseEstimate = new Pose2d(0,0,0);
//        cv.startStreamingDashboard();

        //intakePreviousPos = robot.intake.intakeMotor.getCurrentPosition();

        while (opModeIsActive() && !isStopRequested()) {

//            if (gamepad1.dpad_up) {
//                robot.slides.resetAllEncoders();
//            }
            //intakePreviousPos = robot.checkJam(intakePreviousPos);
            //DRIVE
            switch (currentMode) {
                case NORMAL_CONTROL:
                    if (gamepad1.left_trigger > 0.5) {
                        x = -gamepad1.left_stick_x * (1 - 0.66 * gamepad1.left_trigger);
                        y = -gamepad1.left_stick_y * (1 - 0.66 * gamepad1.left_trigger);
                        rx = gamepad1.right_stick_x * (1 - 0.66 * gamepad1.left_trigger);

                    } else {
                        x = -gamepad1.left_stick_x;
                        y = -gamepad1.left_stick_y;
                        rx = gamepad1.right_stick_x;
                    }
                    robot.setDrivePower(-x, y, rx);
                    break;
                case ALIGN_TO_POINT:
                    Pose2d poseEstimate = robot.drive.getLocalizer().getPoseEstimate();
                    // If x is pressed, we break out of the automatic following
                    if (gamepad2.square) {
                        robot.drive.breakFollowing();
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!robot.drive.isBusy()) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }
                    break;
            }

            //CLAW
            if (gamepad1.cross && !previousCross) {
                robot.smartClawOrderedOpen();
            }

            if (gamepad1.square && !previousSquare) {
                robot.claw.wrist.addAngle(-45);
            }
            if (gamepad1.circle && !previousCircle) {
                robot.claw.wrist.addAngle(45);
            }
            if (gamepad1.right_trigger > 0.75) {
                robot.claw.wrist.addAngle(180);
            }
//            if (gamepad1.circle && !previousCircle) {
//                robot.claw.wrist.setAngle(176);
//            }
            previousCircle = gamepad1.circle;
            previousCross = gamepad1.cross;
            previousSquare = gamepad1.square;


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
                robot.intake.reverse();
            } else if ((gamepad1.left_trigger<0.2)  && (gamepad1.left_trigger != previousLeftTriggerState)){
                robot.intake.stopIntake();
            }
            previousLeftTriggerState = gamepad1.left_trigger;

            //DEPOSIT
            if (gamepad1.left_bumper) {
                robot.depositPreset();
            }

            //SLIDES
            if (gamepad1.dpad_left && !previousDpadLeftState) {
                robot.slides.incrementBackdropTarget(-70);
            } else if (gamepad1.dpad_right && !previousDpadRightState) {
                robot.slides.incrementBackdropTarget(70);
            }
            previousDpadLeftState = gamepad1.dpad_left;
            previousDpadRightState = gamepad1.dpad_right;


            //DRONE
            if (gamepad2.triangle && !previousDroneState) {
                robot.drone.launch();
            }
            previousDroneState = gamepad1.triangle;

            // AUTO ALIGN
            if (gamepad2.square && !previousAutoAlignState && currentMode != Mode.ALIGN_TO_POINT) {
                currentMode = Mode.ALIGN_TO_POINT;
                Trajectory traj1 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0,30, Math.toRadians(270)))
                        .build();

                robot.drive.followTrajectoryAsync(traj1);
            }
            previousAutoAlignState = gamepad2.square;

            if (gamepad2.circle && !previousRelocalizeState) {
                relocalizePoseEstimate = cv.relocalizeUsingBackdrop(robot.drive.getPoseEstimate());
                robot.drive.setPoseEstimate(relocalizePoseEstimate);
            }
            previousRelocalizeState = gamepad2.circle;

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
                buzzers = 2;
            }

            robot.slides.update();
            robot.antiJam();
            double loop = System.nanoTime();

            if (smartIntakeEnabled) {
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

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            sumLoop += 1000000000 / (loop - loopTime);
            loopIterations += 1;
            telemetry.addData("avg hz", (sumLoop / loopIterations));

//            telemetry.addData("SENSOR1 ", robot.intakeSensor.getRangeSensor1());
//            telemetry.addData("SENSOR2 ", robot.intakeSensor.getRangeSensor2());
            telemetry.addData("TIME LEFT: ", ((120-matchTimer.time(TimeUnit.SECONDS))));
            telemetry.addData("SLIDES TARGET: ", robot.slides.target);
            telemetry.addData("SLIDES POSITION: ", robot.slides.getPos());
            telemetry.addData("LEVEL: ", robot.slides.currentLevel);
            loopTime = loop;
            telemetry.update();
        }
    }
}