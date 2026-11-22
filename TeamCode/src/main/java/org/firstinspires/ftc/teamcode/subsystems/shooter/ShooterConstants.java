package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static int autoOffset = -200;
    public static double[] pollenDistances = new double[]{
            33,
            50,
            58,
            72,
            91
    };

    public static double[] pollenFlywheelRpms = new double[]{
            -2800 + autoOffset,
            -2850 + autoOffset,
            -2900 + autoOffset,
            -3000 + autoOffset,
            -3350 + autoOffset
    };

    public static double[] pollenRollerRpms = new double[]{
            4300,
            4300,
            4300,
            4450,
            4500
    };

    public static double[] pollenHoodPoses = new double[]{
            0.47,
            0.5,
            0.52,
            0.55,
            0.56
    };

    public static double[] pollenTOFSeconds = new double[]{
            0.6521,
            0.6521,
            0.726,
            0.726,
            0.85
    };

    public static double[] nectarDistances = new double[]{
            33,
            51,
            59,
            73,
            92
    };

    public static double[] nectarFlywheelRpms = new double[]{
            -3100,
            -3250,
            -3400,
            -3650,
            -3950
    };

    public static double[] nectarRollerRpms = new double[]{
            4400,
            4300,
            4400,
            4450,
            4450
    };

    public static double[] nectarHoodPoses = new double[]{
            .52,
            .56,
            .56,
            .56,
            .58
    };

    public static double[] nectarTOFSeconds = new double[]{

            //
            0.55,
            .6,
            //0.7,
            0.8,
            //0.9,
            0.9,
            1.0
    };

    public static boolean enableSOTM = true;
    public static double offsetSeconds = 0.0875;
}
