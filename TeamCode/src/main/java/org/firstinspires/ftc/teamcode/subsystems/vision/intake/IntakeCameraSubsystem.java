package org.firstinspires.ftc.teamcode.subsystems.vision.intake;

import android.graphics.Color;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.VisionPortal.ColorBlobLocatorProcessorMulti;
import org.firstinspires.ftc.teamcode.VisionPortal.ColorRange;
import org.firstinspires.ftc.teamcode.subsystems.vision.CameraBaseSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

@Config
public class IntakeCameraSubsystem extends CameraBaseSubsystem {
    private final ColorBlobLocatorProcessorMulti intakeProcessor = new ColorBlobLocatorProcessorMulti(
            new ColorRange(ColorSpace.HSV, new Scalar(255, 255, 255), new Scalar(161, 255, 255)),
            org.firstinspires.ftc.teamcode.VisionPortal.ImageRegion.asImageCoordinates(0, 0, 320, 240),
            ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY,
            Color.rgb(255, 120, 31),
            Color.rgb(255, 255, 255),
            Color.rgb(3, 227, 252)
    );

    public static CameraConfig config = new CameraConfig(
            "intakeCam",
            "IntakeCam",
            false,
            320,
            240,
            VisionPortal.StreamFormat.MJPEG,
            new ProcessorConfig(
                    0,
                    true
            )
    );

    public IntakeCameraSubsystem(HardwareMap hardwareMap, int viewId, Telemetry telemetry) {
        super(hardwareMap, viewId, telemetry, config);

        intakeProcessor.addColor(new ColorRange(ColorSpace.HSV, new Scalar(50, 50, 40), new Scalar(95, 255, 255)));
        intakeProcessor.addColor(new ColorRange(ColorSpace.HSV, new Scalar(126, 50, 50), new Scalar(161, 255, 255)));
        intakeProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 500, 1000000));

        buildVisionPortal(intakeProcessor);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
