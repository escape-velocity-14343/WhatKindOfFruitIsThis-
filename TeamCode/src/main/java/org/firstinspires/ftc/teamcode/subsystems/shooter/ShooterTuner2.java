package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static dev.nextftc.units.Units.RotationsPerMinute;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Logger;

@TeleOp
@Config
public class ShooterTuner2 extends LinearOpMode {
    public static double desired_Velocity = 0.0;
    public static double rollerVelocity = 0.0;
    public static double turretPos = 0.0;

    @Override
    public void runOpMode(){
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);
        waitForStart();
        intake.on();
        transfer.intake();
        Logger.start(this);
        while (opModeIsActive()){
            shooter.flywheel.setTargetVelocity(RotationsPerMinute.of(desired_Velocity));
            shooter.rollers.setPower(1);
            shooter.flywheel.periodic();
            turret.setDegrees(turretPos);
            telemetry.addData("Desired Velocity", rollerVelocity);
            telemetry.update();
        }
    }
}
