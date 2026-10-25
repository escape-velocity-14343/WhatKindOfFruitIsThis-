package org.firstinspires.ftc.teamcode.subsystems.shooter.rail;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class RailSubsystem2 extends SubsystemBase {
    public static double railPollenPos = 0.85;
    public static double railNectarPos = 0.90;
    Servo railServo;
    public RailSubsystem2(HardwareMap hwMap){
        railServo = hwMap.get(Servo.class, "railServo");
    }

    public void setPos(double pos){
        railServo.setPosition(pos);

    }

    public double curPos(){
        return railServo.getPosition();
    }

    public void pollen(){
        railServo.setPosition(railPollenPos);
    }
    public void nectar(){
        railServo.setPosition(railNectarPos);
    }
}
