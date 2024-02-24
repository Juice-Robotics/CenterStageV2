package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Claw;

@TeleOp(group = "competition")
@Config
//@Disabled
public class TransferTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, false);

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        robot.slides.resetAllEncoders();
        robot.arm.setAngleArm(90);
        robot.claw.setPositionClaw(150, Claw.Side.BOTH);
        robot.intake.setAngle(176);
        robot.claw.wrist.setAngle(19);
        robot.arm.setAngleElbow(275);
        robot.slides.runToPosition(0);
        boolean previousX = gamepad1.right_bumper;
        boolean previousBumper = gamepad1.left_bumper;
        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.right_bumper && !previousX) {
                Thread thread = new Thread(new Runnable() {
                    public void run() {
                        robot.intake.stopIntake();
                        sleep(500);
                        robot.arm.runtoPreset(Levels.CAPTURE);
                        sleep(500);
                        robot.claw.setClawClose(Claw.Side.BOTH);
                        sleep(500);
                        robot.arm.runtoPreset(Levels.INTERMEDIATE);
                        robot.intake.runToPreset(Levels.INTAKE);
                    }});
                thread.start();
            }

            if (gamepad1.left_bumper && !previousBumper) {
                robot.startIntake();
            }

            previousX = gamepad1.cross;
            previousBumper = gamepad2.left_bumper;
            robot.slides.update();
        }
    }
}


// twist wrist 180
// pivot wrist 90
// axons: 120
//intake 70%