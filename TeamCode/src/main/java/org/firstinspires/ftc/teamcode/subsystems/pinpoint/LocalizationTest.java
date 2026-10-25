package org.firstinspires.ftc.teamcode.subsystems.pinpoint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import gay.zharel.fateweaver.telemetry.Fate;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@Utility
public class LocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        PinpointSubsystem pinpoint = new PinpointSubsystem(hardwareMap);
        MecanumDriveSubsystem mecanum = new MecanumDriveSubsystem(hardwareMap, () -> pinpoint.getHeading().getRadians());

        pinpoint.reset();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.startWasPressed()) {
                pinpoint.reset();
            }

            pinpoint.periodic();

            mecanum.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addData("Pinpoint pose", pinpoint.getPose());
            Fate.log("Pinpoint/Pose", pinpoint.getPose());
            telemetry.update();
        }

    }

}
