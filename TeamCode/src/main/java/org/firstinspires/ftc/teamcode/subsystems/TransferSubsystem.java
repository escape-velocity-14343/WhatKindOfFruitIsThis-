package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
public class TransferSubsystem extends SubsystemBase {

    public static double transferIntakeSpeed = 0.6;
    public static double transferTransferSpeed = 1;
    public static double transferRejectSpeed = -0.3;
    DcMotor transferMotor;
    public TransferSubsystem(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
    }

    public void setPower(double power) {
        transferMotor.setPower(power);
    }

    public void transfer() {
        setPower(transferTransferSpeed);
    }

    public void intake() {
        setPower(transferIntakeSpeed);
    }

    public void reject() {
        setPower(transferRejectSpeed);
    }

    public void off() {
        setPower(0.0);
    }
}
