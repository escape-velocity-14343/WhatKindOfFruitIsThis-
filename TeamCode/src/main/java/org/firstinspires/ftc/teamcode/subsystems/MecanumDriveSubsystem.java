package org.firstinspires.ftc.teamcode.subsystems;


import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.function.DoubleSupplier;

public class MecanumDriveSubsystem extends SubsystemBase {
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DoubleSupplier heading;

    public MecanumDriveSubsystem (HardwareMap hwMap, DoubleSupplier headingSupplier) {
        frontLeftMotor = hwMap.dcMotor.get("front left");
        backLeftMotor = hwMap.dcMotor.get("back left");
        frontRightMotor = hwMap.dcMotor.get("front right");
        backRightMotor = hwMap.dcMotor.get("back right");
        heading = headingSupplier;

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    /**
     * Mecanum drive method for holonomic robot control.
     *
     * @param x    Robot relative FORWARD movement component
     * @param y    Robot relative LEFT movement component
     * @param turn Robot relative rotation component
     */
    public void drive(double x, double y, double turn){

        double botHeading = heading.getAsDouble();

        if (Math.abs(botHeading)> 7) {
            Log.e("MecanumDriveSubsystem", "IMU Heading is out of range: " + botHeading);
        }

        // Rotate the movement direction counter to the bot's rotation
        double rotX = -y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
        double rotY = -y * Math.sin(-botHeading) + x * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double frontLeftPower = (rotY + rotX + turn) / denominator;
        double backLeftPower = (rotY - rotX + turn) / denominator;
        double frontRightPower = (rotY - rotX - turn) / denominator;
        double backRightPower = (rotY + rotX - turn) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
    public void setHeadingSupplier(DoubleSupplier headingSupplier){
        heading = headingSupplier;
    }
}