package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class TurretSubsystem {
    private ServoImplEx turretServo1;
    private ServoImplEx turretServo2;

    public static double offsetDegrees = 170;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretServo1 = (ServoImplEx) hardwareMap.get(Servo.class, "turret1");
        turretServo2 = (ServoImplEx) hardwareMap.get(Servo.class, "turret2");
        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(544, 2472);
        turretServo1.setPwmRange(pwmRange);
        turretServo2.setPwmRange(pwmRange);

    }

    public void setPos(double position){
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
    }

    public void setDegrees(double degrees){
        setPos((degrees + offsetDegrees)/360);
    }
}
