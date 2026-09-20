package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.units.measuretypes.AngularVelocity;
import gay.zharel.fastlane.VoltageCache;
import gay.zharel.fateweaver.flight.FlightRecorder;
import gay.zharel.fateweaver.log.FateLogWriter;
import org.firstinspires.ftc.teamcode.util.Logger;

import static dev.nextftc.units.Units.*;
import static gay.zharel.fastlane.UnitsKt.AppliedThrottle;
import static gay.zharel.fastlane.UnitsKt.Throttle;

@Config
public class ShooterSubsystem2 {
    private DcMotorEx shooterMotorRight;
    private DcMotor shooterMotorLeft;
    private DcMotorEx encoderMotor;

    public ShooterSubsystem2 (HardwareMap hwMap){
        shooterMotorRight = (DcMotorEx) hwMap.get(DcMotor.class, "shooterMotorRight");
        shooterMotorLeft = hwMap.get(DcMotor.class, "shooterMotorLeft");
    }
    public void on_left(){
        shooterMotorLeft.setPower(-1);
    }
    public void off_left(){
        shooterMotorLeft.setPower(0);
    }
    public void on_right() {
        shooterMotorRight.setPower(1);
    }
    public void off_right(){
        shooterMotorRight.setPower(0);
    }
}
