package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.StepperServo;

@TeleOp(group = "competition")
@Config
public class ServoTest extends LinearOpMode {
    public static double CLAW_POS = 177.5;
    public static double ARM_POS = 177.5;
    public static double INTAKE_POS = 177.5;
    public static double ELBOW_POS = 177.5;
    public static double WRIST_POS = 177.5;


    StepperServo one;
    StepperServo two;
    StepperServo three;
    StepperServo four;
    StepperServo five;
    StepperServo six;
    StepperServo seven;
    StepperServo eight;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        one = new StepperServo(0, "arm1", hardwareMap);
        two = new StepperServo(0, "arm2", hardwareMap);
        three = new StepperServo(0, "claw1", hardwareMap);
        four = new StepperServo(0, "claw2", hardwareMap);
        five = new StepperServo(0, "intakeServo1", hardwareMap);
        six = new StepperServo(0, "intakeServo2", hardwareMap);
        seven = new StepperServo(0, "wrist", hardwareMap);
        eight = new StepperServo(0, "elbow", hardwareMap);

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            one.setAngle((float) ARM_POS);
            two.setAngle((float) ARM_POS);
//            three.setAngle((float) CLAW_POS);
//            four.setAngle((float) CLAW_POS);
            five.setAngle((float) INTAKE_POS);
            six.setAngle((float) INTAKE_POS);
            seven.setAngle((float) WRIST_POS);
            eight.setAngle((float) ELBOW_POS);
        }
    }
}


// arm: 90
// claw open (hold): 70
// claw close (drop): 180
// intake lock: 320;
// elbow (capture): 280, wrist 200
// arm intermediate: 100
