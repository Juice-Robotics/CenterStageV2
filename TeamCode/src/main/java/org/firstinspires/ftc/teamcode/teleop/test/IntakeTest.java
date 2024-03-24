package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

@TeleOp(group = "competition")
@Config
//@Disabled
public class IntakeTest extends LinearOpMode {
    DcMotorEx motor;
    public static double MOT_POWER = 0;
    public static double ARM_POS = 90;
    public static double ELBOW = 275;
    public static double WRIST_PIVOT = 19;
    public static double INTAKE_DEPLOY = 176;
    public static double CLAW = 150;



    StepperServo elbow;
    StepperServo wrist2;

    StepperServo arm1;
    StepperServo arm2;

    StepperServo intakeDeploy1;
    StepperServo intakeDeploy2;
    StepperServo claw1;
    StepperServo claw2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbow = new StepperServo(0, "elbow", hardwareMap);
        wrist2 = new StepperServo(0, "wrist", hardwareMap);
        arm1 = new StepperServo(0, "arm1", hardwareMap);
        arm2 = new StepperServo(0, "arm2", hardwareMap);
        intakeDeploy1 = new StepperServo(0, "intakeServo1", hardwareMap);
        intakeDeploy2 = new StepperServo(0, "intakeServo2", hardwareMap);
        claw1 = new StepperServo(0, "claw1", hardwareMap);
        claw2 = new StepperServo(0, "claw2", hardwareMap);

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            motor.setPower(MOT_POWER);
            elbow.setAngle((float) ELBOW);
            wrist2.setAngle((float) WRIST_PIVOT);
            arm1.setAngle((float) ARM_POS);
            arm2.setAngle((float) ARM_POS);
            intakeDeploy1.setAngle((float) INTAKE_DEPLOY);
            intakeDeploy2.setAngle((float) INTAKE_DEPLOY);
            claw1.setAngle((float) CLAW);
            claw2.setAngle((float) CLAW);

            telemetry.addData("current draw", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("velocity", motor.getVelocity());
            telemetry.update();
        }
    }
}


// twist wrist 180
// pivot wrist 90
// axons: 120
//intake 70%