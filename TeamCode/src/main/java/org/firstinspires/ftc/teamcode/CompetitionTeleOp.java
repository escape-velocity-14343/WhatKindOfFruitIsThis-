package org.firstinspires.ftc.teamcode;

import static dev.nextftc.units.Units.RotationsPerMinute;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

import java.util.List;

@TeleOp(name = "TeleOp")
public class CompetitionTeleOp extends LinearOpMode {
    int turretAngle = -5;
    int shooterVelocity = 5000;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);


        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap, () -> pinpoint.getHeading().getRadians());
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(params);


        boolean lastA = false;
        CommandScheduler.getInstance().cancelAll();
        waitForStart();
        ElapsedTime loopTime = new ElapsedTime();
        int loops = 0;
        long initialTime = System.currentTimeMillis();
        while (opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
            //CommandScheduler.
            //CommandScheduler.getInstance().run();
            shooter.turret.setDegrees(turretAngle);
            telemetry.addData("Turret Angle", turretAngle);
            telemetry.addData("Shooter RPM", shooter.flywheel.getCurrentVelocity()
                    .into(RotationsPerMinute));
            telemetry.addData("Shooter Target RPM", shooterVelocity);
            //pinpoint.update();
            drive.driveFC(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            if (gamepad1.left_bumper) {
                intake.setPower(-0.5);
                transfer.reject();
            }
            else if (gamepad1.right_trigger > 0.5 && Math.abs(shooter.flywheel.getCurrentVelocity().into(RotationsPerMinute) - shooterVelocity) < 280) {
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
                imu.resetYaw();
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
            telemetry.addData("pinpoint heading", pinpoint.getHeading().getDegrees());
            //telemetry.addData("shooter velocity", shooter.flywheel.getCurrentVelocity().into(RotationsPerMinute));
            loops++;
            telemetry.addData("current loop ms", loopTime.milliseconds());
            telemetry.addData("running average ms", ((double)System.currentTimeMillis() - initialTime) / ((double) loops));
            telemetry.update();

            loopTime.reset();

            shooter.flywheel.periodic();
            shooter.rollers.periodic();

        }
    }
}
