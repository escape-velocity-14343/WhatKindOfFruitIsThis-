package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class ColorSensorSubsystem extends SubsystemBase {
    public double proximityThreshold = 2;
    RevColorSensorV3 ballSensor;
    //boolean isPollen;

    public ColorSensorSubsystem(HardwareMap hwMap){
        ballSensor = hwMap.get(RevColorSensorV3.class, "ballSensor");
    }
    public boolean proximityDetected(){
        return ballSensor.getDistance(DistanceUnit.CM) < proximityThreshold;
    }

    public boolean yellowDetected(){
        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(ballSensor.red(), ballSensor.green(), ballSensor.blue(), hsv);
        return 20 < hsv[0] && hsv[0] > 30;
    }

    //@Override
    //public void periodic(){
    //    if (proximityDetected()){
    //        isPollen = yellowDetected();
    //    }
    //}
}
