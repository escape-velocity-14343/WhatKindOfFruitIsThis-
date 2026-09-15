package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap, () -> 0.0);

        boolean lastA = false;
        waitForStart();
        while (opModeIsActive()) {
            drive.drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.right_bumper) {
                intake.on();
            } else {
                intake.off();
            }

            shooter.setPower(gamepad1.right_trigger);

            if (gamepad1.left_bumper) {
                transfer.on();
            }
            else {
                transfer.off();
            }

            lastA = gamepad1.a;
        }
    }
}
