package org.firstinspires.ftc.teamcode.subsystems.lib;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SquIDController {
    double p, i, d;

    public SquIDController() {
        p = 0;
        i = 0;
        d = 0;
    }

    public void setPID(double p) {
        this.p = p;
    }

    public double calculate(double setpoint, double current) {
        return Math.sqrt(Math.abs((setpoint - current))) * p * Math.signum(setpoint - current);
    }
    public double calculateAngleWrapping(double setpoint, double current)  {
        return calculate(AngleUnit.normalizeDegrees(setpoint-current), 0);
    }
}