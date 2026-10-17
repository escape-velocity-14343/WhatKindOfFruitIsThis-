package org.firstinspires.ftc.teamcode;

import static dev.nextftc.units.Units.RotationsPerMinute;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem2;
import org.firstinspires.ftc.teamcode.util.Logger;

@TeleOp(name = "TeleOp")
public class CompetitionTeleOp extends LinearOpMode {
    int turretAngle = -5;
    int shooterVelocity = 3700;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Logger.start(this);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);

        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap, () -> pinpoint.getHeading().getRadians());
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);


        boolean lastA = false;
        CommandScheduler.getInstance().cancelAll();
        waitForStart();
        while (opModeIsActive()) {
            //CommandScheduler.
            //CommandScheduler.getInstance().run();
            turret.setDegrees(turretAngle);
            telemetry.addData("Turret Angle", turretAngle);

            drive.driveFC(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, pinpoint.getHeading().getRadians());

            if (gamepad1.left_bumper) {
                intake.setPower(-0.5);
                transfer.reject();
            }
            else if (gamepad1.right_trigger > 0.5) {
                    transfer.transfer();
                    intake.setPower(1);
            }
            else {
                    intake.on();
                    transfer.off();
            }
            if (gamepad1.dpadLeftWasReleased()) {
                turretAngle -= 10;
                if (turretAngle < -45){
                    turretAngle = -45;
                }

            if (gamepad1.dpadRightWasReleased()){
                turretAngle += 10;
                if (turretAngle > 45){
                    turretAngle = 45;
                }
            }
            }
            if (gamepad1.options) {
                pinpoint.resetIMU();
            }

            //if (gamepad1.right_trigger > 0.05) {
            //    transfer.reject();
            //} else

            if (gamepad1.dpadUpWasReleased()){
                shooterVelocity += 100;
                if (shooterVelocity > 6000){
                    shooterVelocity = 6000;
                }
            } else if (gamepad1.dpadDownWasReleased()) {
                shooterVelocity -= 100;
            }

            if (gamepad1.right_bumper) {
                shooter.flywheel.setTargetVelocity(RotationsPerMinute.of(shooterVelocity));
                shooter.rollers.setPower(1);
            } else {
                shooter.flywheel.setTargetVelocity(RotationsPerMinute.of(0));
                shooter.rollers.setPower(0);
            }

            //lastA = gamepad1.a;
            telemetry.update();
            shooter.flywheel.periodic();
            shooter.rollers.periodic();

        }
    }
}
