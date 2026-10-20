package org.firstinspires.ftc.teamcode.subsystems.shooter.hood;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import dev.nextftc.units.Measure;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.unittypes.AngleUnit;
import gay.zharel.fateweaver.telemetry.Fate;

import static dev.nextftc.units.Units.Degrees;

@Config
public class HoodSubsystem extends SubsystemBase {

    // we define 0 degrees as the top of the hood being parallel with the plane of the shooter
    // another way to say it: all the way down, pointing straight up

    // 90 degrees is pointing straight fowards

    public static double hoodMaxDegrees = 35.0;
    public static double hoodMinDegrees = 22.0;

    private final Servo hoodServo;
    private Angle angle = Degrees.of(hoodMinDegrees);

    public HoodSubsystem(HardwareMap hardwareMap) {
        this.hoodServo = hardwareMap.servo.get("hoodServo");

        // reverse so that 0.0 is all the way down
        this.hoodServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setServo(double pos) {
        this.hoodServo.setPosition(pos);
    }

    public void setAngle(Angle angle) {
        setServo(getServoValueFromAngle(angle));

        this.angle = angle;

        Fate.log("Shooter/Hood/Degrees", angle.into(Degrees));
        Fate.log("Shooter/Hood/ServoValue", hoodServo.getPosition());
    }

    private double getServoValueFromAngle(Angle angle) {
        Angle minAngle = Degrees.of(hoodMinDegrees);
        Angle maxAngle = Degrees.of(hoodMaxDegrees);

        return angle
                .minus(minAngle)
                .div((Measure<? extends AngleUnit>) maxAngle.minus(minAngle));
    }

    @Override
    public void periodic() {
        Fate.log("Shooter/Hood/Degrees", angle.into(Degrees));
        Fate.log("Shooter/Hood/ServoValue", hoodServo.getPosition());
    }

}
