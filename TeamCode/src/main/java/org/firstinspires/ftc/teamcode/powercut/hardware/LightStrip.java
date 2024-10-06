package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LightStrip {
    private RevBlinkinLedDriver driver = null;
    private RevBlinkinLedDriver.BlinkinPattern pattern = null;

    public void init(@NonNull HardwareMap hardwareMap) {
        driver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }

    public void confetti() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.CONFETTI;
        driver.setPattern(pattern);
    }
}
