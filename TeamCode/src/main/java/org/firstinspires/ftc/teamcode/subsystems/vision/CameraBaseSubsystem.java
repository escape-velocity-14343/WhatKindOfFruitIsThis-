package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Log;
import android.util.Size;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import gay.zharel.fateweaver.telemetry.Fate;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.Arrays;
import java.util.Objects;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

public abstract class CameraBaseSubsystem extends SubsystemBase {

    public static class CameraConfig {
        public String hardwareMapName, subsystemName;
        public boolean enableLiveView;
        public int width, height;
        public VisionPortal.StreamFormat format;
        public ProcessorConfig[] configs;

        public CameraConfig(String hardwareMapName, String subsystemName, boolean enableLiveView,
                            int width, int height, VisionPortal.StreamFormat format, ProcessorConfig... configs
                            ) {
            this.hardwareMapName = hardwareMapName;
            this.subsystemName = subsystemName;
            this.enableLiveView = enableLiveView;
            this.width = width;
            this.height = height;
            this.format = format;
            this.configs = configs;
        }
    }

    public static class ProcessorConfig {
        public boolean isProcessorEnabled;
        public int index;

        public ProcessorConfig(int index, boolean isProcessorEnabled) {
            this.isProcessorEnabled = isProcessorEnabled;
            this.index = index;
        }
    }

    protected VisionPortal visionPortal;
    protected final VisionPortal.Builder visionPortalBuilder;
    protected final Telemetry telemetry;
    protected final CameraConfig config;
    protected final VisionProcessor[] processors;
    private boolean lastLiveView = false;

    public CameraBaseSubsystem(HardwareMap hardwareMap, int viewId, Telemetry telemetry, CameraConfig config, VisionProcessor... processors) {
        this.telemetry = telemetry;
        this.config = config;
        this.processors = processors;

        visionPortalBuilder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, config.hardwareMapName))
                .setCameraResolution(new Size(config.width, config.height))
                .setStreamFormat(config.format)
                .enableLiveView(config.enableLiveView)
                .setShowStatsOverlay(false)
                .setLiveViewContainerId(viewId);

        lastLiveView = config.enableLiveView;
    }

    protected void buildVisionPortal(VisionProcessor... processors) {
        visionPortalBuilder.addProcessors(processors);
        visionPortal = visionPortalBuilder.build();
    }

    protected String getFateLogName() {
        return "Vision/" + config.subsystemName;
    }

    @Override
    public void periodic() {
        Fate.log( getFateLogName() + "/FPS", visionPortal.getFps());
        Fate.log(getFateLogName() + "/Status", visionPortal.getCameraState().toString());

        if (Objects.nonNull(visionPortal)) {
            if (lastLiveView != config.enableLiveView) {
                setEnableLiveView(config.enableLiveView);
                lastLiveView = config.enableLiveView;
            }

            // for each processor, enable based on config
            for (ProcessorConfig pconfig : config.configs) {
                if (pconfig.isProcessorEnabled != visionPortal.getProcessorEnabled(processors[pconfig.index])) {
                    setProcessor(processors[pconfig.index], pconfig.isProcessorEnabled);
                }
            }
        }
    }

    protected void setProcessor(VisionProcessor processor, boolean enabled) {
        if (visionPortal == null) {
            Fate.log(getFateLogName() + "/Warn", "Apriltag portal is null");
            return;
        }
        visionPortal.setProcessorEnabled(processor, enabled);
    }

    public void stopVisionStreaming() {
        if (visionPortal == null) {
            Fate.log(getFateLogName() + "/Warn", "Apriltag portal is null");
            return;
        }
        visionPortal.stopStreaming();
    }

    public void resumeVisionStreaming() {
        if (visionPortal == null) {
            Fate.log(getFateLogName() + "/Warn", "Apriltag portal is null");
            return;
        }
        visionPortal.resumeStreaming();
    }

    public void setEnableLiveView(boolean set) {
        if (set) {
            visionPortal.resumeLiveView();
        } else {
            visionPortal.stopLiveView();
        }
    }

    public boolean waitForStreaming(VisionPortal portal, long timeoutMs) {
        long startMs = System.currentTimeMillis();
        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING
                && System.currentTimeMillis() - startMs < timeoutMs) {
            telemetry.addLine("waiting for camera to stream: " + portal.getCameraState());
            telemetry.update();
        }
        return portal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public boolean setExposure(int exposureMicroseconds) {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Log.i("vision", "exiting early because not streaming");
            return false;
        }
        ExposureControl control = visionPortal.getCameraControl(ExposureControl.class);
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

    public boolean waitForSetExposure(int exposureMicroseconds, long timeoutMs, int maxAttempts) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            Log.i("vision", String.format("Attempting to set %s camera exposure, attempt %d, %d ms after start", config.subsystemName, attempts, msAfterStart));
            if (setExposure(exposureMicroseconds)) {
                Log.i("vision", "Set exposure succeeded");
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }
        Log.e("vision", "Set exposure failed");
        return false;
    }
}
