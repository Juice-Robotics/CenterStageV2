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
    public static double ARM_POS = 70;
    public static double ELBOW_POS = 285;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, false);

        // Initialize your own robot class
        waitForStart();
        Thread thread = new Thread(new Runnable() {
            public void run() {
                robot.startIntake();
                sleep(2000);
                robot.intake.stopIntake();
                robot.intake.intakeMotor.setSpeed(0.6F);
                sleep(500);
                robot.arm.setAngleArm(ARM_POS);
                robot.arm.setAngleElbow(ELBOW_POS);
                robot.claw.setClawClose(Claw.Side.BOTH);
                sleep(800);
                robot.arm.setAngleArm(140);
                robot.intake.runToPreset(Levels.INTAKE);
                robot.intake.intakeMotor.setSpeed(0);
            }});
        thread.start();

        if (isStopRequested()) return;
        boolean previousX = gamepad1.right_bumper;
        boolean previousBumper = gamepad1.left_bumper;
        while (opModeIsActive() && !isStopRequested()) {

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