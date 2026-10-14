package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class TurretSubsystem {
    private Servo turretServo1;
    private Servo turretServo2;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretServo1 = hardwareMap.get(Servo.class, "");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
    }

    public void setPos(double position){
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
    }

    public void setDegrees(double degrees){
        setPos(degrees/360);
    }
}
