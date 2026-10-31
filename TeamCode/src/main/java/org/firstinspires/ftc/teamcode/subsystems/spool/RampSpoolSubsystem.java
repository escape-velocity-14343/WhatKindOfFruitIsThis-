package org.firstinspires.ftc.teamcode.subsystems.spool;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class RampSpoolSubsystem extends SubsystemBase {
    private Servo spoolServo;

    public static double raisedPos = 0.5;
    public static double loweredPos = 1;

    public enum RampState {
        RAISED,
        LOWERED,
        UNKOWN
    }

    private RampState state = RampState.UNKOWN;

    public RampSpoolSubsystem(HardwareMap hardwareMap) {
        spoolServo = hardwareMap.get(Servo.class, "flowerServo");
    }

    public void lower() {
        spoolServo.setPosition(loweredPos);
        state = RampState.LOWERED;
    }
    public void raise() {
        spoolServo.setPosition(raisedPos);
        state = RampState.RAISED;
    }

    public double getPos() {
        return spoolServo.getPosition();
    }
    public void setPos(double position) {
        spoolServo.setPosition(position);
    }
}