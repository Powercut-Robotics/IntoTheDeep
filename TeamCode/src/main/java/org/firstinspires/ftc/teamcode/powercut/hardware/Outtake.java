package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.powercut.settings;

public class Outtake {
    public ServoImplEx leftArm, rightArm, grip;

    public void init(HardwareMap hardwareMap) {
        leftArm = hardwareMap.get(ServoImplEx.class, "leftArm");
        rightArm = hardwareMap.get(ServoImplEx.class, "rightArm");
        grip = hardwareMap.get(ServoImplEx.class, "grip");

        leftArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        grip.setPwmRange(new PwmControl.PwmRange(500, 2500));

        leftArm.setDirection(ServoImplEx.Direction.REVERSE);
    }

    public class RelaxSystem implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftArm.setPwmDisable();
            rightArm.setPwmDisable();
            grip.setPwmDisable();
            return false;

        }
    }

    public Action relaxSystem() {
        return new RelaxSystem();
    }

    // ARM
    public class DepositSampArm implements Action {
        private long startTime = 0;
        private static final long DURATION = 1000;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            leftArm.setPosition(settings.upperArmSampDeposit);
            rightArm.setPosition(settings.upperArmSampDeposit);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action depositSampArm() {
        return new DepositSampArm();
    }

    public class DepositSpecArm implements Action {
        private long startTime = 0;
        private static final long DURATION = 1000;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            leftArm.setPosition(settings.upperArmSpecDeposit);
            rightArm.setPosition(settings.upperArmSpecDeposit);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action depositSpecArm() {
        return new DepositSpecArm();
    }

    public class SpecIntakeArm implements Action {
        private long startTime = 0;
        private static final long DURATION = 1000;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            leftArm.setPosition(settings.upperArmIntake);
            rightArm.setPosition(settings.upperArmIntake);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action specIntakeArm() {
        return new SpecIntakeArm();
    }

    public class TravelArm implements Action {
        private long startTime = 0;
        private static final long DURATION = 500;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            leftArm.setPosition(settings.upperArmTravel);
            rightArm.setPosition(settings.upperArmTravel);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action travelArm() {
        return new TravelArm();
    }

    public class TransferArm implements Action {
        private long startTime = 0;
        private static final long DURATION = 1000;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            leftArm.setPosition(settings.upperArmTransfer);
            rightArm.setPosition(settings.upperArmTransfer);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action transferArm() {
        return new TransferArm();
    }

    // GRIP
    public class CloseGrip implements Action {
        private long startTime = 0;
        private static final long DURATION = 500;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            grip.setPosition(settings.gripClosed);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action closeGrip() {
        return new CloseGrip();
    }

    public class OpenGrip implements Action {
        private long startTime = 0;
        private static final long DURATION = 300;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            grip.setPosition(settings.gripOpen);

            long elapsedTime = System.currentTimeMillis() - startTime;
            return elapsedTime < DURATION;
        }
    }

    public Action openGrip() {
        return new OpenGrip();
    }

    public class RelaxGrip implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            grip.setPwmDisable();

            return false;
        }
    }

    public Action relaxGrip() {
        return new RelaxGrip();
    }
}
