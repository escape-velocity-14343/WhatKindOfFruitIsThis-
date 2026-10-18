package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import gay.zharel.fateweaver.flight.FlightRecorder;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class ShooterSubsystem2 {
    private DcMotorEx shooterMotorRight;
    private DcMotorEx shooterMotorLeft;
    private DcMotorEx encoderMotor;

    public ShooterSubsystem2 (HardwareMap hwMap){
        shooterMotorRight = (DcMotorEx) hwMap.get(DcMotor.class, "flywheelMotor");
        shooterMotorLeft = (DcMotorEx) hwMap.get(DcMotor.class, "counterrollerMotor");
    }

    public void setPower(double power){
        shooterMotorRight.setPower(power);
        shooterMotorLeft.setPower(-power);

    }
    //public void on_left(){
    //    shooterMotorLeft.setPower(0.5);
    //}
    //public void off_left(){
    //    shooterMotorLeft.setPower(0);
    //}
    public void on_right() {
        shooterMotorRight.setPower(-0.5);
    }

    public double get_right_current() {
        return shooterMotorRight.getCurrent(CurrentUnit.AMPS);
    }

    //public double get_left_current() {
    //    return shooterMotorLeft.getCurrent(CurrentUnit.AMPS);
    //}
    public void off_right(){
        shooterMotorRight.setPower(0);
    }
}
