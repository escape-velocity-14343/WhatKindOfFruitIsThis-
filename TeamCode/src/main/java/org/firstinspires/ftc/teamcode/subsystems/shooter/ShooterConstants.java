package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static double[] pollenDistances = new double[]{
            33.0,
            36.0,
            55.0,
            73.0,
            86.0
    };

    public static double[] pollenFlywheelRpms = new double[]{
            -2600.0,
            -2600.0,
            -2500.0,
            -2900.0,
            -3100.0,
    };

    public static double[] pollenRollerRpms = new double[]{
            4300,
            4700,
            4300,
            4500,
            5000,
    };

    public static double[] pollenHoodPoses = new double[]{
            0.585,
            0.6,
            0.4,
            0.4,
            0.4,
    };

    public static double[] pollenTOFSeconds = new double[]{
            0.6521,
            0.6521,
            0.726,
            0.85,
            0.85
    };

    public static double[] nectarDistances = new double[]{
            48,
            51,
            55, // 9 / 11
            67, //not very good
            93, // absolutely not
            ////
            //37.3,
            //45.7,
            //53.0,
            //58.0,
            //68.0,
            //75.0
    };

    public static double[] nectarFlywheelRpms = new double[]{
            -3100,
            -3100,
            -2900,
            -3400,
            -3900,
            ////
            //-2800.0,
            //-2800.0,
            //-3000.0,
            //-3025.0,
            //-3100.0,
            //-3300.0
    };

    public static double[] nectarRollerRpms = new double[]{
            4300,
            4700,
            4300,
            4500,
            5000,
    };

    public static double[] nectarHoodPoses = new double[]{
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,
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
