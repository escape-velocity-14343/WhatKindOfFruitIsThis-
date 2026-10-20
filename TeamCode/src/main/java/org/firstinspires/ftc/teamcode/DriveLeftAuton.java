package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@Autonomous
public class DriveLeftAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap, ()->0);
        waitForStart();
        sleep(25000);
        drive.drive(0.2, 0.5, 0);
        sleep(500);
        drive.drive(0,0,0);

    }
}
