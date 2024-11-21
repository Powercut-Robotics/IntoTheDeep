package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LightSystem {
    private RevBlinkinLedDriver driver = null;
    private RevBlinkinLedDriver.BlinkinPattern pattern = null;

    public void init(@NonNull HardwareMap hardwareMap) {
        driver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }

    public void confetti() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.CONFETTI;
        driver.setPattern(pattern);
    }
    public void red() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        driver.setPattern(pattern);
    }
    public void green() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        driver.setPattern(pattern);
    }

    public void blue() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        driver.setPattern(pattern);
    }

    public void greyLarson() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_GRAY;
        driver.setPattern(pattern);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.pattern = pattern;
        driver.setPattern(pattern);
    }
}
