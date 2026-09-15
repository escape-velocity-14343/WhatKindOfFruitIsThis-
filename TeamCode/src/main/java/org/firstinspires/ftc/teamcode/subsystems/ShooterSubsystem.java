package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {
    DcMotor shooterMotor;
    boolean on = false;
    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "intake");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public void on() {
        setPower(1.0);
        on = true;
    }

    public void off() {
        setPower(0.0);
        on = false;
    }

    public void toggle() {
        if (on) {
            off();
        }
        else {
            off();
        }
    }

}
