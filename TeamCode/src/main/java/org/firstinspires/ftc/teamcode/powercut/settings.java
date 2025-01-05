package org.firstinspires.ftc.teamcode.powercut;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;

@Config
public class settings {
    public static PIDCoefficientsEx liftCoefficients = new PIDCoefficientsEx(0.0025,0.00001,0.0015, 500, 5, 0.05);
    public static double liftEqCoef = 0.015;
    public static double liftHoldPower = 0.5;

    public static int liftTopBasket = 3000;
    public static int liftBottomBasket = 1250;
    public static int liftTopRung = 1500;
    public static int liftTopRungAttached = 1400;
    public static int liftBottomRung = 1250;
    public static int liftBottomRungAttached = 1150;
    public static int liftRetraction = 25;

    public static int allowableExtensionDeficit= 50;

    public static double intakeArmTransfer = 0.45;
    public static double intakeArmIntake = 0.8;

    public static double extendoIntake = 0.7;
    public static double extendoTransfer = 0.5;

    public static double upperArmDeposit = 0.45;
    public static double upperArmIntake = 0.6;
    public static double upperArmTransfer = 0.8;

    public static double gripClosed = 0.63;
    public static double gripOpen = 0.3;

    public static double basketAlignDistance = 15;
    public static PIDCoefficientsEx basketXYCoefficients = new PIDCoefficientsEx(0.1,0,0,0,0,0);
    public static PIDCoefficientsEx basketYawCoefficients = new PIDCoefficientsEx(-0.1,0,0,0,0,0);

    public static double colourThreshMultiplier = 1.5;

    public static double driveCacheAmount = 0.01;
}
