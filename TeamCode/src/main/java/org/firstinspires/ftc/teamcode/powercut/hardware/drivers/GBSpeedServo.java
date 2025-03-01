package org.firstinspires.ftc.teamcode.powercut.hardware.drivers;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class GBSpeedServo {
    public ServoImplEx servo;
    private final double degreesOfFreedom = 300;
    private final double sixtyDegreeTime = 0.11;
    private final double fullRotationPeriod = ((degreesOfFreedom/60) * (sixtyDegreeTime * 1000));

    public double position;
    private double distanceToMove;
    public double eta;
    private double startMove = 0;
    private double duration;

    public void init(ServoImplEx servo) {
        servo.setPwmRange(new PwmControl.PwmRange(500,2500));
        this.servo = servo;
    }

    public void setPosition(double newPosition) {
        double oldPosition = this.getPosition();
        distanceToMove = newPosition - oldPosition;
        startMove = System.currentTimeMillis();
        duration = Math.abs(distanceToMove) * fullRotationPeriod;
        eta = startMove + duration;

        servo.setPosition(newPosition);
    }

    public boolean isPositionIncreasing() {
        return distanceToMove > 0;
    }

    public boolean isMoving() {
        return System.currentTimeMillis() < eta;
    }

    public double getPosition() {
        double timeMoved = System.currentTimeMillis() - startMove;
        double amountMoved = timeMoved/duration;

        if (amountMoved > 1) {
            position = servo.getPosition();
            distanceToMove = 0;
        } else {
            position = servo.getPosition() - (distanceToMove - (distanceToMove * amountMoved));
        }

        return position;
    }
}
