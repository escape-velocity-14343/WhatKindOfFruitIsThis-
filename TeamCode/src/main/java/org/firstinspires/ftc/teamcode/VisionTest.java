/* Copyright (c) 2024 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.VisionPortal.ColorBlobLocatorProcessorMulti;
import org.firstinspires.ftc.teamcode.VisionPortal.ColorRange;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

import java.util.concurrent.TimeUnit;

/**
 * This OpMode demonstrates the basics of using multiple vision portals simultaneously
 */
@TeleOp

public class VisionTest extends LinearOpMode
{
    VisionPortal portal1;
    VisionPortal portal2;
    VisionPortal portal3;

    AprilTagProcessor aprilTagProcessor1;
    ColorBlobLocatorProcessorMulti intakeProcesor;
    ColorBlobLocatorProcessorMulti transferProcessor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Because we want to show two camera feeds simultaneously, we need to inform
        // the SDK that we want it to split the camera monitor area into two smaller
        // areas for us. It will then give us View IDs which we can pass to the individual
        // vision portals to allow them to properly hook into the UI in tandem.
        int[] viewIds = VisionPortal.makeMultiPortalView(3, VisionPortal.MultiPortalLayout.VERTICAL);

        // We extract the two view IDs from the array to make our lives a little easier later.
        // NB: the array is 2 long because we asked for 2 portals up above.
        int portal1ViewId = viewIds[0];
        int portal2ViewId = viewIds[1];
        int portal3ViewId = viewIds[2];

        // If we want to run AprilTag detection on two portals simultaneously,
        // we need to create two distinct instances of the AprilTag processor,
        // one for each portal. If you want to see more detail about different
        // options that you have when creating these processors, go check out
        // the ConceptAprilTag OpMode.
        aprilTagProcessor1 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setNumThreads(2)
                .build();
        aprilTagProcessor1.setDecimation(2.0f);
        intakeProcesor = new ColorBlobLocatorProcessorMulti(
                new ColorRange(ColorSpace.HSV, new Scalar(255, 255, 255), new Scalar(161, 255, 255)),
                org.firstinspires.ftc.teamcode.VisionPortal.ImageRegion.asImageCoordinates(0, 0, 320, 240),
                ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY,
                Color.rgb(255, 120, 31),
                Color.rgb(255, 255, 255),
                Color.rgb(3, 227, 252)
        );
        intakeProcesor.addColor(new ColorRange(ColorSpace.HSV, new Scalar(50, 50, 40), new Scalar(95, 255, 255)));
        intakeProcesor.addColor(new ColorRange(ColorSpace.HSV, new Scalar(126, 50, 50), new Scalar(161, 255, 255)));
        intakeProcesor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 500, 1000000));


        transferProcessor = new ColorBlobLocatorProcessorMulti(
                new ColorRange(ColorSpace.HSV, new Scalar(255, 255, 255), new Scalar(161, 255, 255)),
                org.firstinspires.ftc.teamcode.VisionPortal.ImageRegion.asImageCoordinates(0, 0, 320, 240),
                ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY,
                Color.rgb(255, 120, 31),
                Color.rgb(255, 255, 255),
                Color.rgb(3, 227, 252)
        );
        transferProcessor.addColor(new ColorRange(ColorSpace.HSV, new Scalar(50, 50, 40), new Scalar(95, 255, 255)));
        transferProcessor.addColor(new ColorRange(ColorSpace.HSV, new Scalar(126, 50, 50), new Scalar(161, 255, 255)));
        transferProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 500, 1000000));


        // Now we build both portals. The CRITICAL thing to notice here is the call to
        // setLiveViewContainerId(), where we pass in the IDs we received earlier from
        // makeMultiPortalView().
        portal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "shooterCam"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                .setShowStatsOverlay(false)
                .setLiveViewContainerId(portal1ViewId)
                .addProcessor(aprilTagProcessor1)
                .build();
        portal2 = new VisionPortal.Builder()
                .addProcessors(intakeProcesor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "intakeCam"))
                .enableLiveView(false)
                .setShowStatsOverlay(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setLiveViewContainerId(portal2ViewId)
                .build();
        portal3 = new VisionPortal.Builder()
                .addProcessors(transferProcessor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "transferCam"))
                .enableLiveView(false)
                .setShowStatsOverlay(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setLiveViewContainerId(portal3ViewId)
                .build();
        while (portal1.getCameraState()!= VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("My name is Quincy and I love food.");
            telemetry.update();
        }
        waitForSetExposure(670, 670, 10000, portal1);
        waitForSetExposure(670, 670, 4800000, portal2);
        waitForSetExposure(670, 670, 4800000, portal3);
        waitForStart();
        ElapsedTime elapsedTime = new ElapsedTime();

        // Main Loop
        while (opModeIsActive())
        {
            // Just show some basic telemetry to demonstrate both processors are working in parallel
            // on their respective cameras. If you want to see more detail about the information you
            // can get back from the processor, you should look at ConceptAprilTag.
            telemetry.addData("Number of tags in Camera 1", aprilTagProcessor1.getDetections().size());
            telemetry.addData("AprilTag FPS", portal1.getFps());
            telemetry.addData("Intake FPS", portal2.getFps());
            telemetry.addData("Transfer FPS", portal3.getFps());
            telemetry.addData("Loop Time (ms)", elapsedTime.milliseconds());
            if (gamepad1.dpad_up) {
                portal1.stopLiveView();
                portal2.stopLiveView();
                portal3.stopLiveView();
            }
            if (gamepad1.dpad_down) {
                portal1.resumeLiveView();
                portal2.resumeLiveView();
                portal3.resumeLiveView();
            }
            if (gamepad1.dpad_right) {
                portal1.setProcessorEnabled(aprilTagProcessor1, false);
            }
            if (gamepad1.dpad_left) {
                portal1.setProcessorEnabled(aprilTagProcessor1, true);
            }
            telemetry.update();
            elapsedTime.reset();
        }
    }
    public boolean setExposure(int exposure, VisionPortal visionPortal) {
        if (visionPortal.getCameraState()!= VisionPortal.CameraState.STREAMING) {
            Log.i("intake camera", "exiting early because not streaming");
            return false;
        }
        ExposureControl control = visionPortal.getCameraControl(ExposureControl.class);
        control.setMode(ExposureControl.Mode.Manual);
        Log.i("intake camera", "exposure before: " + control.getExposure(TimeUnit.MICROSECONDS));
        Log.i("intake camera", "Camera Mode: " + control.getMode() + " Exposure supported: " + control.isExposureSupported() + " Exposure min: " + control.getMinExposure(TimeUnit.MICROSECONDS) + " Exposure Max: " + control.getMaxExposure(TimeUnit.MICROSECONDS));
        boolean worked = control.setExposure(exposure, TimeUnit.MICROSECONDS);
        Log.i("intake camera", "exposure after: " + control.getExposure(TimeUnit.MICROSECONDS));
        return worked;
    }


    public boolean waitForSetExposure(long timeoutMs, int maxAttempts, int exposure, VisionPortal portal) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            Log.i("camera", String.format("Attempting to set camera exposure, attempt %d, %d ms after start", attempts, msAfterStart));
            if (setExposure(exposure, portal)) {
                Log.i("camera", "Set exposure succeeded");
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }

        Log.e("camera", "Set exposure failed");
        return false;
    }
}
