package org.firstinspires.ftc.teamcode.subsystems.intake;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.*;
import gay.zharel.fateweaver.telemetry.Fate;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double PIVOT_UP = 0.0;
    public static double PIVOT_DOWN = 0.0;

    public enum PivotState {
        UP(PIVOT_UP),
        DOWN(PIVOT_DOWN);
        public double pos;
        PivotState(double pos) {};
    }

    private PivotState pivotState = PivotState.UP;

    private final DcMotorEx intakeMotor;
    private Servo intakePivot;
    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // intakePivot = hardwareMap.servo.get("intake pivot");
    }

    public void periodic() {
        // this is usually cached so its prob fine
        Fate.log("Intake/IntakeThrottle", intakeMotor.getPower());
        Fate.log("Intake/Pivot", pivotState);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
        Fate.log("Intake/IntakeThrottle", power);
    }
    public void setPivot(PivotState state) {
        this.pivotState = state;
        // intakePivot.setPosition(state.pos);
        Fate.log("Intake/Pivot", state);
    }

    public void on() {
        setPower(0.7);
    }
    public void off() {
        setPower(0.0);
    }
    public void reverse() {
        setPower(-0.7);
    }
    public void down() {
        setPivot(PivotState.DOWN);
    }
    public void up() {
        setPivot(PivotState.UP);
    }

    public Command commandOn() {
        return new InstantCommand(this::on, this).alongWith(
                new InstantCommand(this::down)
        );
    }

    public Command commandOff() {
        return new InstantCommand(this::off, this).alongWith(
                new InstantCommand(this::up)
        );
    }

    public Command commandReverse() {
        return new InstantCommand(this::reverse, this);
    }
}
