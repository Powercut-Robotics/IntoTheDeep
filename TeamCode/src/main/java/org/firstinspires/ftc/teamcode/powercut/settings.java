package org.firstinspires.ftc.teamcode.powercut;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;

@Config
public class settings {
    public static PIDCoefficientsEx liftCoefficients = new PIDCoefficientsEx(0.00015,0.00001,0.001, 500, 5, 0.05);
    public static double liftEqCoef = 0.015;
    public static double liftHoldPower = 0.05;

    public static int liftTopBasket = 5500;
    public static int liftBottomBasket = 2500;
    public static int liftTopRung = 3000;
    public static int liftTopRungAttached = 2800;
    public static int liftBottomRung = 10;
    public static int liftBottomRungAttached = 10;
    public static int liftRetraction = 0;

    public static int allowableExtensionDeficit= 50;

    public static double armRaised = 0.45;
    public static double armDeposit = 0.475;
    public static double armLowered = 0.8;

    public static double gripClosed = 0.63;
    public static double gripOpen = 0.3;

    public static int redThresh = 2000;
    public static int blueThresh = 2000;
    public static int[] yellowThresh = {1500, 1500};
    public static int intakeDistThresh = 100;
    public static double driveCacheAmount = 0.01;
}
