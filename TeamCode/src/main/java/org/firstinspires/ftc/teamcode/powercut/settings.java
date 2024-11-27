package org.firstinspires.ftc.teamcode.powercut;

import com.acmerobotics.dashboard.config.Config;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

@Config
public class settings {
    public static PIDCoefficients liftCoefficients = new PIDCoefficients(0,0,0);
    public static double liftEqCoef = 0.015;

    public static int liftTopBasket = 10;
    public static int liftBottomBasket = 10;
    public static int liftTopRung = 6500;
    public static int liftTopRungAttached = 10;
    public static int liftBottomRung = 10;
    public static int liftBottomRungAttached = 10;
    public static int liftRetraction = 0;

    public static int allowableExtensionDeficit= 10;

    public static double armRaised = 0.5;
    public static double armDeposit = 0.6;
    public static double armLowered = 0.8;

    public static double gripClosed = 0.5;
    public static double gripOpen = 0.6;

    public static int redThresh = 100;
    public static int blueThresh = 100;
    public static int[] yellowThresh = {100, 100};
    public static double driveCacheAmount = 0.01;
}
