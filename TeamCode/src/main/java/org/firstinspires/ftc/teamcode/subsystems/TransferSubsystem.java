package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TransferSubsystem extends SubsystemBase {
    DcMotor transferMotor;
    public TransferSubsystem(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
    }

    public void setPower(double power) {
        transferMotor.setPower(power);
    }

    public void on() {
        setPower(1.0);
    }

    public void off() {
        setPower(0.0);
    }
}
