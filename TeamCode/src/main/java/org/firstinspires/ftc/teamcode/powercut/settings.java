package org.firstinspires.ftc.teamcode.powercut;

import com.acmerobotics.dashboard.config.Config;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

@Config
public class settings {
    public static PIDCoefficients liftCoefficients = new PIDCoefficients(0,0,0);

    public static int liftTopBasket = 10;
    public static int liftBottomBasket = 10;
    public static int liftTopRung = 10;
    public static int liftTopRungAttached = 10;
    public static int liftBottomRung = 10;
    public static int liftBottomRungAttached = 10;
    public static int liftRetraction = 0;
    public static int allowableExtensionDeficit= 10;

    public static double armRaised = 0;
    public static double armDeposit = 0.1;
    public static double armLowered = 0.5;

    public static double gripClosed = 0;
    public static double gripOpen = 0.1;

    public static double driveCacheAmount = 0.01;
}
