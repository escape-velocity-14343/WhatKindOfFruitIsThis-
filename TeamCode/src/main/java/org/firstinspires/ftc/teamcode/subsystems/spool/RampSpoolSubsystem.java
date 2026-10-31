package org.firstinspires.ftc.teamcode.subsystems.spool;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class RampSpoolSubsystem extends SubsystemBase {
    private Servo spoolServo;

    public static double raisedPos = 0.2;
    public static double loweredPos = 0.7;

    public enum RampState {
        RAISED,
        LOWERED,
        UNKOWN
    }

    private RampState state = RampState.UNKOWN;

    public RampSpoolSubsystem(HardwareMap hardwareMap) {
        spoolServo = hardwareMap.get(Servo.class, "ramp_spool_servo");
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