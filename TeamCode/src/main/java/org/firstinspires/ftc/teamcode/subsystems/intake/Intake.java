package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.Motor;
import org.firstinspires.ftc.teamcode.lib.MotorEx;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class Intake {

    public StepperServo intakeServo1;
    public StepperServo intakeServo2;

    public float OFFSET = 0;

    public float intakeDown = 176 - OFFSET;
    public float autoIntake = 200 - OFFSET;


    public float intakeUp = 315 - OFFSET;

    public MotorEx intakeMotor;

    public Intake(StepperServo intakeServo1, StepperServo intakeServo2, MotorEx intakeMotor) {
        this.intakeServo1 = intakeServo1;
        this.intakeServo2 = intakeServo2;
        this.intakeMotor = intakeMotor;
        intakeMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void startIntake(){
        intakeMotor.setSpeed(1);
        intakeServo1.setAngle(intakeDown);
        intakeServo2.setAngle(intakeDown);
    }
    public void autoStartIntake(){
        intakeServo1.setAngle(autoIntake);
        intakeServo2.setAngle(autoIntake);
    }

    public void stopIntake(){
        intakeMotor.setSpeed(0);
        intakeServo1.setAngle(intakeUp);
        intakeServo2.setAngle(intakeUp);
    }

    public void reverseIntake(){
        intakeServo1.setAngle(intakeDown);
        intakeServo2.setAngle(intakeDown);
        intakeMotor.setSpeed(-0.6F);
    }

    public void reverseIntakeSpike(){
        intakeServo1.setAngle(intakeDown);
        intakeServo2.setAngle(intakeDown);
        intakeMotor.setSpeed(-0.3F);
    }

    public void setAngle(float angle) {
        intakeServo1.setAngle(angle);
        intakeServo2.setAngle(angle);
    }

    public void runToPreset(Levels level) {
        if (level == Levels.ZERO) {
            setAngle(0);
        } else if (level == Levels.INTAKE) {
            setAngle(intakeDown);
        } else if (level == Levels.INTERMEDIATE) {
            setAngle(intakeUp);
        } else if (level == Levels.CLIMB_EXTEND) {
            setAngle(intakeUp);
        } else if (level == Levels.INIT) {
            setAngle(intakeUp);
        }
    }

    public void reverse(){
        intakeMotor.setSpeed(-0.6F);
    }

}