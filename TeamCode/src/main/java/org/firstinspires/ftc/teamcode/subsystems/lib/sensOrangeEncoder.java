package org.firstinspires.ftc.teamcode.subsystems.lib;



import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class sensOrangeEncoder {

    private AnalogInput sensor;
    private double offset;
    private boolean inverted;
    private double gearRatio = 1.0;

    private double lastVoltage = 0;

    {
        // default to 0
        offset = 0;

        // default to false
        inverted = false;
    }

    public sensOrangeEncoder(String key, HardwareMap hMap) {
        this.sensor = hMap.analogInput.get(key);
    }

    public void setInverted(boolean inverted){
        this.inverted = inverted;
    }

    /**
     * Sets offset of the encoder
     */
    public void setPositionOffset(double offset){
        this.offset = offset;
    } 
    
    /**
     * @return Degrees
     */
    public double getDegrees(){
        double voltage = sensor.getVoltage();

        lastVoltage = voltage;
        return AngleUnit.normalizeDegrees((inverted ? -1 : 1) * (voltage-0.043)/3.1*360 * gearRatio + offset);
    }
    /**
    * @return who would know
    */
    public double getRadians() {
        return Math.toRadians(getDegrees());
    }
    public Rotation2d getRotation2d() {
        return new Rotation2d(getRadians());
    }
    public void setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
    }


}