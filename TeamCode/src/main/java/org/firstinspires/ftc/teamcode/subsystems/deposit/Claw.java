package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class Claw {
    public StepperServo depositServoLeft;
    public StepperServo depositServoRight;
    public StepperServo wrist;

    public ClawStatus isOpenLeft = ClawStatus.OPEN; // if open, true
    public ClawStatus isOpenRight = ClawStatus.OPEN; // if open, true

    // CONSTANTS
    public float clawOpen = 150;
    public float clawClose = 60; //smaller = tighter

    public Claw(StepperServo depositServo1, StepperServo depositServo2,  StepperServo wrist) {
        this.depositServoLeft = depositServo1;
        this.depositServoRight = depositServo2;
        this.wrist = wrist;
        // this.wrist.servo.setDirection(Servo.Direction.REVERSE);
    }

    public void toggle(Side side) {
        switch (side) {
            case LEFT:
                if (isOpenLeft == ClawStatus.OPEN) {
                    this.depositServoLeft.servo.setPosition(clawClose);
                    isOpenLeft = ClawStatus.CLOSED;
                } else {
                    this.depositServoLeft.servo.setPosition(clawOpen);
                    isOpenLeft = ClawStatus.OPEN;
                }
                break;
            case RIGHT:
                if (isOpenRight == ClawStatus.OPEN) {
                    this.depositServoRight.servo.setPosition(clawClose);
                    isOpenRight = ClawStatus.CLOSED;
                } else {
                    this.depositServoRight.servo.setPosition(clawOpen);
                    isOpenRight = ClawStatus.OPEN;
                }
                break;
            case BOTH:
                // TAKE LEFT CLAW STATUS TO STANDARDIZE
                if (isOpenLeft == ClawStatus.OPEN) {
                    this.depositServoLeft.servo.setPosition(clawClose);
                    isOpenLeft = ClawStatus.CLOSED;
                    this.depositServoRight.servo.setPosition(clawClose);
                    isOpenRight = ClawStatus.CLOSED;
                } else {
                    this.depositServoLeft.servo.setPosition(clawOpen);
                    isOpenLeft = ClawStatus.OPEN;
                    this.depositServoRight.servo.setPosition(clawOpen);
                    isOpenRight = ClawStatus.OPEN;
                }
                break;
        }
    }

    public void setPositionClaw(float angle, Side side) {
        switch (side) {
            case LEFT:
                this.depositServoLeft.setAngle(angle);
                break;
            case RIGHT:
                this.depositServoRight.setAngle(angle);
            case BOTH:
                this.depositServoLeft.setAngle(angle);
                this.depositServoRight.setAngle(angle);
        }
    }

    public void setPositionWrist(float rotation) {
        this.wrist.setAngle(rotation);
    }

    public void runToWristPreset(Levels level) {
        if (level == Levels.ZERO) {
            setPositionWrist(0);
        } else if (level == Levels.INTAKE) {
            setPositionWrist(19);
        } else if (level == Levels.DEPOSIT) {
            setPositionWrist(119);
        }
    }

    public void setClawOpen(Side side) {
        switch (side) {
            case LEFT:
                this.depositServoLeft.setAngle(clawOpen);
                isOpenLeft = ClawStatus.OPEN;
                break;
            case RIGHT:
                this.depositServoRight.setAngle(clawOpen);
                isOpenRight = ClawStatus.OPEN;
                break;
            case BOTH:
                this.depositServoLeft.setAngle(clawOpen);
                this.depositServoRight.setAngle(clawOpen);
                isOpenRight = ClawStatus.OPEN;
                isOpenLeft = ClawStatus.OPEN;
                break;
        }
    }

    public void setClawOpen() {
        this.depositServoLeft.setAngle(clawOpen);
        this.depositServoRight.setAngle(clawOpen);
        isOpenRight = ClawStatus.OPEN;
        isOpenLeft = ClawStatus.OPEN;
    }

    public void setClawClose(Side side) {
        switch (side) {
            case LEFT:
                this.depositServoLeft.setAngle(clawClose);
                isOpenLeft = ClawStatus.CLOSED;
                break;
            case RIGHT:
                this.depositServoRight.setAngle(clawClose);
                isOpenRight = ClawStatus.CLOSED;
                break;
            case BOTH:
                this.depositServoLeft.setAngle(clawClose);
                this.depositServoRight.setAngle(clawClose);
                isOpenRight = ClawStatus.CLOSED;
                isOpenLeft = ClawStatus.CLOSED;
                break;
        }
    }

    public void setClawClose() {
        this.depositServoLeft.setAngle(clawClose);
        this.depositServoRight.setAngle(clawClose);
        isOpenRight = ClawStatus.CLOSED;
        isOpenLeft = ClawStatus.CLOSED;
    }

    public enum ClawStatus {
        OPEN,
        CLOSED,
        ZERO
    }

    public enum Side {
        LEFT,
        RIGHT,
        BOTH
    }

}
