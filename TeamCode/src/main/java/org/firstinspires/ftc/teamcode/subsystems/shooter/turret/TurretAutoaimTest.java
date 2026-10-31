package org.firstinspires.ftc.teamcode.subsystems.shooter.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import gay.zharel.fateweaver.telemetry.Fate;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

import static dev.nextftc.units.Units.Degrees;

@Utility
public class TurretAutoaimTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);
        MecanumDriveSubsystem mecanum = new MecanumDriveSubsystem(hardwareMap, () -> pinpoint.getHeading().getRadians());

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.startWasPressed()) {
                pinpoint.reset();
            }

            pinpoint.periodic();

            mecanum.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
            shooter.update(pinpoint.getPose(), telemetry);

            telemetry.addData("target shooter degrees", shooter.state.turretAngle.into(Degrees));
            telemetry.addData("Pinpoint pose", pinpoint.getPose());
            Fate.log("Pinpoint/Pose", pinpoint.getPose());

            telemetry.update();
        }
    }
}
