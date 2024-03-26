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

    public float intakeDown = 177 - OFFSET;
    public float autoIntake = 209 - OFFSET; // -10 increments


    public float intakeUp = 320 - OFFSET;
    public int intakeUpMotorPosition = 0;

    public MotorEx intakeMotor;
    public int target;

    public Intake(StepperServo intakeServo1, StepperServo intakeServo2, MotorEx intakeMotor) {
        this.intakeServo1 = intakeServo1;
        this.intakeServo2 = intakeServo2;
        this.intakeMotor = intakeMotor;
        intakeMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void startIntake(){
        setSpeed(1);
        intakeServo1.setAngle(intakeDown);
        intakeServo2.setAngle(intakeDown);
    }
    public void autoStartIntake(){
        intakeServo1.setAngle(autoIntake);
        intakeServo2.setAngle(autoIntake);
    }

    public void stopIntake(){
        setSpeed(0);
        runToIntakePositionGlobal(intakeUpMotorPosition);
        intakeServo1.setAngle(intakeUp);
        intakeServo2.setAngle(intakeUp);
    }

    public void reverseIntake(){
        intakeServo1.setAngle(intakeDown);
        intakeServo2.setAngle(intakeDown);
        setSpeed(-0.6F);
    }

    public void reverseIntakeSpike(){
        intakeServo1.setAngle(intakeDown);
        intakeServo2.setAngle(intakeDown);
        setSpeed(-0.3F);
    }

    public void setAngle(float angle) {
        intakeServo1.setAngle(angle);
        intakeServo2.setAngle(angle);
    }

    public float getAngle() {
        return intakeServo1.getAngleDeg();
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

    public void runToIntakePosition(int ticks) {
        if (intakeMotor.motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            intakeMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        intakeMotor.motor.setTargetPosition(ticks);
        intakeMotor.motor.setPower(-1);
    }

    public void setSpeed(float power) {
        if (intakeMotor.motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            intakeMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        intakeMotor.setSpeed(power);
    }

    /**
    * This function accounts for the wrapovers for the intake when it completes a full revolution, and will go to the nearest tick position to achieve the same physical intake position
     */
    public void runToIntakePositionGlobal(int ticks) {
        // math.floor gets us how many revolutions we went thru, and we multiply by 28 to get the tick position of the lowest position in the rev
        target = (int) (Math.floor(intakeMotor.motor.getCurrentPosition() / 141.351724138) * 141.351724138) + ticks;

        runToIntakePosition(target);
    }

    public void resetAllEncoders(){
        intakeMotor.resetEncoder();
    }

    public void reverse(){
        intakeMotor.setSpeed(-0.6F);
    }

}