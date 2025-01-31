package org.firstinspires.ftc.teamcode.powercut;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;

@Config
public class settings {
    public static PIDCoefficientsEx liftCoefficients = new PIDCoefficientsEx(0.0035,0.001,0.00005, 500, 150, 0);
    public static PIDCoefficientsEx liftHangCoefficients = new PIDCoefficientsEx(0.008,0.001,0.00000, 1000, 0, 0);
    public static double liftEqCoef = 0.00025;
    public static double liftHoldPower = 0.05;
    public static double liftHangPower = -0.2;

    public static int liftTopBasket = 2900;
    public static int liftBottomBasket = 1250;
    public static int liftTopRung = 1500;
    public static int liftTopRungAttached = 1300;
    public static int liftBottomRung = 1250;
    public static int liftBottomRungAttached = 1050;
    public static int liftPreHang = 500;
    public static int liftRetraction = 0;

    public static int allowableExtensionDeficit= 50;

    public static double intakeArmSafe = 0.97;
    public static double intakeArmTransfer = 1.00;
    public static double intakeArmIntake = 0.325;

    public static double extendoIntake = 0.23;
    public static double extendoTravel = 0.475;
    public static double extendoTransfer = 0.505;
    public static double extendoClearance = 0.4;

    public static double upperArmDeposit = 0.85;
    public static double upperArmTravel = 0.5;
    public static double upperArmIntake = 1.0;
    public static double upperArmTransfer = 0.0;

    public static double gripClosed = 0.7;
    public static double gripOpen = 0.5;

    public static double basketAlignDistance = 25;
    public static double rungAlignDistance = 25;
    public static double subAlignDistance = 25;
    public static double wallAlignDistance = 25;
    public static PIDCoefficientsEx basketXYCoefficients = new PIDCoefficientsEx(0.05,0,0.005,0,100,0.25);
    public static PIDCoefficientsEx rungYCoefficients = new PIDCoefficientsEx(0.025,0,0.02,0,100,0);
    public static PIDCoefficientsEx basketYawCoefficients = new PIDCoefficientsEx(-1.5,0,0.005,0,0.5,0);
    public static PIDCoefficientsEx yawLockCoefficients = new PIDCoefficientsEx(-1.25,0,0.5,0,0.5,0);
    public static PIDCoefficientsEx subYCoefficients = new PIDCoefficientsEx(-0.025,0,0.02,0,100,0);



    public static double colourThreshMultiplier = 1.5;

    public static double driveCacheAmount = 0.01;
}
