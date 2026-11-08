package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.graphics.Color;
import android.util.Log;
import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.subsystems.lib.VisionPortal.ColorBlobLocatorProcessorMulti;
import org.firstinspires.ftc.teamcode.subsystems.lib.VisionPortal.ColorRange;
import org.firstinspires.ftc.teamcode.subsystems.lib.VisionPortal.ImageRegion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

import java.util.concurrent.TimeUnit;

import gay.zharel.fateweaver.telemetry.Fate;

public class ApriltagSubsystem extends SubsystemBase {

    AprilTagProcessor aprilTagProcessor;
    VisionPortal atagPortal;
    // ---------------------------------------------------------------------
    // Vision setup / helpers (ported from VisionTest)
    // ---------------------------------------------------------------------

    public ApriltagSubsystem(HardwareMap hardwareMap, int viewId, boolean enableLiveView) {
        //int[] viewIds = VisionPortal.makeMultiPortalView(3, VisionPortal.MultiPortalLayout.VERTICAL);

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setNumThreads(1)
                .build();
        aprilTagProcessor.setDecimation(2.0f);

        atagPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "shooterCam"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(enableLiveView)
                .setShowStatsOverlay(false)
                .setLiveViewContainerId(viewId)
                .addProcessor(aprilTagProcessor)
                .build();

        setEnableLiveView(false);
    }

    // log: fps, detections

    public void setVisionProcessing(boolean enabled) {
        if (atagPortal == null) {
            Fate.log("Vision/AprilTag/Warn", "Apriltag portal is null");
            return;
        }
        atagPortal.setProcessorEnabled(aprilTagProcessor, enabled);
    }

    public void stopVisionStreaming() {

        atagPortal.stopStreaming();
    }

    private void resumeVisionStreaming() {
        if (shooterPortal == null) {
            return;
        }
        shooterPortal.resumeStreaming();
        intakePortal.resumeStreaming();
        transferPortal.resumeStreaming();
    }

    private void setEnableLiveView(boolean set) {
        if (set) {
            shooterPortal.resumeLiveView();
            intakePortal.resumeLiveView();
            transferPortal.resumeLiveView();
        }
        else {
            shooterPortal.stopLiveView();
            intakePortal.stopLiveView();
            transferPortal.stopLiveView();
        }
    }

    private boolean waitForStreaming(VisionPortal portal, long timeoutMs) {
        long startMs = System.currentTimeMillis();
        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING
                && System.currentTimeMillis() - startMs < timeoutMs) {
            telemetry.addLine("waiting for camera to stream: " + portal.getCameraState());
            telemetry.update();
        }
        return portal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    private boolean setExposure(VisionPortal portal, int exposureMicroseconds) {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Log.i("vision", "exiting early because not streaming");
            return false;
        }
        ExposureControl control = portal.getCameraControl(ExposureControl.class);
        if (control == null) {
            Log.i("vision", "no exposure control");
            return false;
        }
        control.setMode(ExposureControl.Mode.Manual);
        Log.i("vision", "exposure before: " + control.getExposure(TimeUnit.MICROSECONDS));
        Log.i("vision", "Camera Mode: " + control.getMode()
                + " Exposure supported: " + control.isExposureSupported()
                + " Exposure min: " + control.getMinExposure(TimeUnit.MICROSECONDS)
                + " Exposure Max: " + control.getMaxExposure(TimeUnit.MICROSECONDS));
        boolean worked = control.setExposure(exposureMicroseconds, TimeUnit.MICROSECONDS);
        Log.i("vision", "exposure after: " + control.getExposure(TimeUnit.MICROSECONDS));
        return worked;
    }

    private boolean waitForSetExposure(VisionPortal portal, int exposureMicroseconds, long timeoutMs, int maxAttempts) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            Log.i("vision", String.format("Attempting to set camera exposure, attempt %d, %d ms after start", attempts, msAfterStart));
            if (setExposure(portal, exposureMicroseconds)) {
                Log.i("vision", "Set exposure succeeded");
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }
        Log.e("vision", "Set exposure failed");
        return false;
    }
}
