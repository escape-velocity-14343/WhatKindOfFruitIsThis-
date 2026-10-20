package org.firstinspires.ftc.teamcode.subsystems.shooter.rail;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import dev.nextftc.units.Measure;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.unittypes.AngleUnit;
import gay.zharel.fateweaver.telemetry.Fate;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

import static dev.nextftc.units.Units.Degrees;

@Config
public class RailSubsystem extends SubsystemBase {
    public static double railPollenPos = 0.0;
    public static double railNectarPos = 1.0;

    private ShooterSubsystem.ShooterElement element = ShooterSubsystem.ShooterElement.POLLEN;
    private final Servo railServo;

    public RailSubsystem(HardwareMap hardwareMap) {
        this.railServo = hardwareMap.servo.get("railServo");
    }

    /**
     * warning: will make the element logging invalid
     */
    public void setServo(double pos) {
        this.railServo.setPosition(pos);
    }

    public void pollen() {
        this.element = ShooterSubsystem.ShooterElement.POLLEN;
        setServo(railPollenPos);
    }

    public void nectar() {
        this.element = ShooterSubsystem.ShooterElement.NECTAR;
        setServo(railNectarPos);
    }

    public void set(ShooterSubsystem.ShooterElement element) {
        switch (element) {
            case POLLEN -> pollen();
            case NECTAR -> nectar();
        }
    }

    @Override
    public void periodic() {
        Fate.log("Shooter/Rail/Element", element.name());
    }

}
