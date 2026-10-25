package org.firstinspires.ftc.teamcode.subsystems.shooter.hood;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

@Config
public class HoodSubsystem2 extends SubsystemBase {
    public static double maxAngle = 0.57;
    public static double minAngle = 0.7;
    Servo hoodServo;

    public HoodSubsystem2(HardwareMap hwMap){
        hoodServo = hwMap.get(ServoImpl.class, "hood");
    }

    public void setPos(double pos){
        if (pos>0.7){pos = 0.7;}
        else if (pos < 0.57){pos = 0.57;}
        hoodServo.setPosition(pos);
    }

    public void max(){hoodServo.setPosition(maxAngle);}
    public void min(){hoodServo.setPosition(minAngle);}
}
