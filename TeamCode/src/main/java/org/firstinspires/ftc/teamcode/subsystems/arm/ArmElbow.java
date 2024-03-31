package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class ArmElbow {
    public StepperServo arm1;
    public StepperServo arm2;
    public StepperServo elbow;

    public double currentAngle;

    // TARGETS
    public double OFFSET = 0;
    public double intermediateTargetArm = 97;
    public double intermediateTargetElbow = 8; //-272
    public double captureTargetArm = 80;
    public double captureTargetElbow = 13;

    public double depositTargetArm = 245;
    public double depositTargetElbow = 145;

    public double initPosArm = 200;
    public double initPosElbow = 60;



    public ArmElbow(StepperServo arm1, StepperServo arm2, StepperServo elbow) {
        this.arm1 = arm1;
        this.arm2 = arm2;
        this.elbow = elbow;
    }

    public void setAngleArm(double angle) {
        arm1.setAngle((float) angle);
        arm2.setAngle((float) angle);
    }

    public void setAngleElbow(double angle) {
        elbow.setAngle((float) angle);
    }

    public double getAngle() {
        return currentAngle;
    }

    public void runtoPreset(Levels level){
        if (level == Levels.CAPTURE) {
            this.setAngleArm(captureTargetArm);
            this.setAngleElbow(captureTargetElbow);
        } else if (level == Levels.DEPOSIT) {
            this.setAngleArm(depositTargetArm);
            this.setAngleElbow(depositTargetElbow);
        }
        else if (level == Levels.INTERMEDIATE) {
            this.setAngleArm(intermediateTargetArm);
            this.setAngleElbow(intermediateTargetElbow);
        }
        else if (level == Levels.CLIMB_EXTEND) {
            this.setAngleArm(160);
            this.setAngleElbow(237-OFFSET);
        }
        else if (level == Levels.INIT) {
            this.setAngleArm(initPosArm);
            this.setAngleElbow(initPosElbow);
        }
    }

}
