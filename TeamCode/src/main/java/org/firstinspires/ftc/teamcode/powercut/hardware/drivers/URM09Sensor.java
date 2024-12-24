package org.firstinspires.ftc.teamcode.powercut.hardware.drivers;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "URM09 Ultrasonic Sensor", description = "DFRobot URM09 Ultrasonic Sensor", xmlTag = "URM09")
public class URM09Sensor extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // I2C Registers
    private static final int REG_DEVICE_ADDRESS = 0x00;
    private static final int REG_PRODUCT_ID = 0x01;
    private static final int REG_VERSION_NUMBER = 0x02;
    private static final int REG_DISTANCE_HIGH = 0x03;
    private static final int REG_DISTANCE_LOW = 0x04;
    private static final int REG_TEMPERATURE_HIGH = 0x05;
    private static final int REG_TEMPERATURE_LOW = 0x06;
    private static final int REG_CONFIGURE = 0x07;
    private static final int REG_COMMAND = 0x08;

    // Default I2C Address
    public static final I2cAddr DEFAULT_ADDRESS = I2cAddr.create7bit(0x11);

    public URM09Sensor(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(DEFAULT_ADDRESS);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.DFRobot;
    }

    @Override
    public String getDeviceName() {
        return "DFRobot URM09 Ultrasonic Sensor";
    }

    /**
     * Set a new I2C address for the sensor.
     * @param newAddress The new address (0x08 to 0x77).
     */
    public void setDeviceAddress(int newAddress) {
        if (newAddress < 0x08 || newAddress > 0x77) {
            throw new IllegalArgumentException("Address must be between 0x08 and 0x77");
        }
        deviceClient.write8(REG_DEVICE_ADDRESS, newAddress);
    }

    /**
     * Read the product ID to verify the sensor.
     * @return The product ID (should be 0x01).
     */
    public int getProductID() {
        return deviceClient.read8(REG_PRODUCT_ID);
    }

    /**
     * Read the version number of the sensor.
     * @return The version number.
     */
    public int getVersionNumber() {
        return deviceClient.read8(REG_VERSION_NUMBER);
    }

    /**
     * Get the distance in centimeters.
     * @return Distance in cm.
     */
    public int getDistance() {
        byte[] data = deviceClient.read(REG_DISTANCE_HIGH, 2);
        return TypeConversion.byteArrayToShort(data) & 0xFFFF;
    }

    /**
     * Get the temperature in Celsius.
     * @return Temperature in Celsius.
     */
    public double getTemperature() {
        byte[] data = deviceClient.read(REG_TEMPERATURE_HIGH, 2);
        int rawTemp = TypeConversion.byteArrayToShort(data) & 0xFFFF;
        return rawTemp / 10.0;
    }

    /**
     * Set the measurement mode of the sensor.
     * @param automatic True for automatic measurement mode, false for passive mode.
     */
    public void setMeasurementMode(boolean automatic) {
        int config = automatic ? 0x80 : 0x00;
        deviceClient.write8(REG_CONFIGURE, config);
    }

    /**
     * Trigger a single measurement in passive mode.
     */
    public void triggerMeasurement() {
        deviceClient.write8(REG_COMMAND, 0x01);
    }

    @Override
    protected synchronized boolean doInitialize() {
        // Default to passive measurement mode
        this.setMeasurementMode(false);
        return true;
    }
}
