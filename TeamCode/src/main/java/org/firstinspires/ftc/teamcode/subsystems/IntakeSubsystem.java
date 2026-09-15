package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    DcMotor intakeMotor;
    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "shooter");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void on() {
        setPower(1.0);
    }

    public void off() {
        setPower(0.0);
    }
}
