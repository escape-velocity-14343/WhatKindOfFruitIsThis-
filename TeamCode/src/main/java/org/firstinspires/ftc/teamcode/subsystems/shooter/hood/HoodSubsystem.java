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

    public static double hoodMaxDegrees = 0.4;
    public static double hoodMinDegrees = 0.58;

    public static double offset = 0.04;

    private final Servo hoodServo;
    private final Servo hoodServo2;
    private Angle angle = Degrees.of(hoodMinDegrees);

    public HoodSubsystem(HardwareMap hardwareMap) {
        this.hoodServo = hardwareMap.servo.get("hoodServo");
        this.hoodServo2 = hardwareMap.servo.get("hoodServo2");

        // reverse so that 0.0 is all the way down
        this.hoodServo.setDirection(Servo.Direction.FORWARD);
        this.hoodServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setServo(double pos) {
        this.hoodServo.setPosition(pos);
        this.hoodServo2.setPosition(pos + offset);
    }

    @Override
    public void periodic() {
        Fate.log("Shooter/Hood/ServoValue", hoodServo.getPosition());
    }

}
