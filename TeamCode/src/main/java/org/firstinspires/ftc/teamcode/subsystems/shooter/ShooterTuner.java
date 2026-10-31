package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.CounterrollerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.FlywheelBaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.flywheel.FlywheelSubsystem;

import java.util.List;
import java.util.Objects;

import static dev.nextftc.units.Units.Degrees;
import static dev.nextftc.units.Units.RotationsPerMinute;

@Utility
@Config

public class ShooterTuner extends LinearOpMode {

    public static double flywheelPowerOrRpm = 0.0;
    public static double counterrollerPowerOrRpm = 0.0;
    public static double hoodDegrees = 0.0;
    public static double turretDegrees = 0.0;
    public static ShooterSubsystem.ShooterElement element = ShooterSubsystem.ShooterElement.POLLEN;
    public static boolean enableIntakeTransfer = true;
    private IntakeSubsystem intake = null;
    private TransferSubsystem transfer = null;
    public static boolean isPollen = false;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);

        if (enableIntakeTransfer) {
            intake = new IntakeSubsystem(hardwareMap);
            transfer = new TransferSubsystem(hardwareMap, () -> isPollen);
        }

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();

            if (FlywheelSubsystem.config.mode == FlywheelBaseSubsystem.FlywheelMode.POWER_ONLY) {
                shooter.flywheel.setPower(flywheelPowerOrRpm);
            } else {
                shooter.flywheel.setTargetVelocity(RotationsPerMinute.of(flywheelPowerOrRpm));
            }

            if (CounterrollerSubsystem.config.mode == FlywheelBaseSubsystem.FlywheelMode.POWER_ONLY) {
                shooter.rollers.setPower(counterrollerPowerOrRpm);
            } else {
                shooter.rollers.setTargetVelocity(RotationsPerMinute.of(counterrollerPowerOrRpm));
            }

            if (enableIntakeTransfer) {
                intake.on();
                transfer.transfer();
            } else if (!Objects.isNull(intake)) {
                intake.off();
                transfer.off();
            }

            shooter.hood.setAngle(Degrees.of(hoodDegrees));
            shooter.turret.setDegrees(turretDegrees);
            //shooter.rail.set(element);

            telemetry.addData("Flywheel RPM", shooter.flywheel.getCurrentVelocity().into(RotationsPerMinute));
            telemetry.addData("Target Flywheel RPM", flywheelPowerOrRpm);

            telemetry.addData("Counterroller RPM", shooter.rollers.getCurrentVelocity().into(RotationsPerMinute));
            telemetry.addData("Target Counterroller RPM", counterrollerPowerOrRpm);

            telemetry.update();
        }
    }

}
