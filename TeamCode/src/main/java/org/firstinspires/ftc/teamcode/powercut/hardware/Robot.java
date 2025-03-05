package org.firstinspires.ftc.teamcode.powercut.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    private final SafeAncillary ancillary = new SafeAncillary();
    private final Lift lift = new Lift();
    private final Drivetrain drive = new Drivetrain();
    private final LightSystem light = new LightSystem();

    private boolean isInitialised = false;

    public void init(HardwareMap hardwareMap) {
        ancillary.init(hardwareMap);
        lift.init(hardwareMap);
        drive.init(hardwareMap);
        light.init(hardwareMap);

        isInitialised = true;
    }

    public SafeAncillary getAncillary() {
        if (!isInitialised) {
            throw new RuntimeException("Not initalised");
        }

        return ancillary;
    }

    public Lift getLift() {
        if (!isInitialised) {
            throw new RuntimeException("Not initalised");
        }

        return lift;
    }

    public Drivetrain getDrive() {
        if (!isInitialised) {
            throw new RuntimeException("Not initalised");
        }

        return drive;
    }

    public LightSystem getLight() {
        if (!isInitialised) {
            throw new RuntimeException("Not initalised");
        }

        return light;
    }

}
