package org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import gay.zharel.fastlane.VoltageCache;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.FlywheelBaseSubsystem;

@Config
public class FlywheelSubsystem extends FlywheelBaseSubsystem {
    public static FlywheelConfiguration config = new FlywheelConfiguration(
            0.0,
            0.0,
            0.0,
            0.0,
            1.0 / 28.0,
            FlywheelMode.POWER_ONLY,
            "Flywheel"
    );
    public FlywheelSubsystem(HardwareMap hardwareMap) {
        super(config,
                hardwareMap.get(DcMotorEx.class, "flywheelMotor"));

        this.flywheelMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

        VoltageCache.init(hardwareMap);
    }

}
