package org.firstinspires.ftc.teamcode.subsystems.lib;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ThaiVPController {
    //Hello please give me the coconut shrimp in physics tomorrow, thanks
    double p, i, d, power;

    public ThaiVPController() {
        p = 0;
        i = 0;
        d = 0;
    }

    public void setPID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public void setPID(double p) {
        this.p = p;
    }

    public void setExponent(double power) {
        this.power = power;
    }

    public double calculate(double setpoint, double current) {
        return Math.pow(Math.abs((setpoint - current)), power) * p * Math.signum(setpoint - current);
    }
    public double calculateAngleWrapping(double setpoint, double current)  {
        return calculate(AngleUnit.normalizeDegrees(setpoint-current), 0);
    }
}
