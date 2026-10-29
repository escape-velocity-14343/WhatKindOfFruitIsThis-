package org.firstinspires.ftc.teamcode.subsystems.lib.VisionPortal;

import static android.graphics.Bitmap.createBitmap;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.Log;

import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;

public class ColorBlobLocatorProcessorMulti extends ColorBlobLocatorProcessor implements VisionProcessor, CameraStreamSource
{
    private org.firstinspires.ftc.teamcode.subsystems.lib.VisionPortal.ColorRange colorRange;
    private org.firstinspires.ftc.teamcode.subsystems.lib.VisionPortal.ImageRegion roiImg;
    private Rect roi;
    private int frameWidth;
    private int frameHeight;
    private Mat roiMat_userColorSpace;
    private final int contourCode;

    private Mat mask = new Mat();

    private final Paint boundingRectPaint;
    private final Paint roiPaint;
    private final Paint contourPaint;

    private final Object lockFilters = new Object();
    private final List<BlobFilter> filters = new ArrayList<>();
    private volatile BlobSort sort;

    private volatile ArrayList<Blob> userBlobs = new ArrayList<>();
    private ArrayList<ColorRange> colors = new ArrayList<>();
    private Mat temp = new Mat();

    private Circle circle = new Circle(0, 0, 0);
    ElapsedTime timer = new ElapsedTime();

    private AtomicReference<Bitmap> lastFrame = new AtomicReference<>(createBitmap(1, 1, Bitmap.Config.RGB_565));



    public ColorBlobLocatorProcessorMulti(ColorRange colorRange, ImageRegion roiImg, ContourMode contourMode,
                                   @ColorInt int boundingBoxColor, @ColorInt int roiColor, @ColorInt int contourColor)
    {
        this.colorRange = colorRange;

        this.roiImg = roiImg;

        if (contourMode == ContourMode.EXTERNAL_ONLY)
        {
            contourCode = Imgproc.RETR_EXTERNAL;
        }
        else
        {
            contourCode = Imgproc.RETR_LIST;
        }

        boundingRectPaint = new Paint();
        boundingRectPaint.setAntiAlias(true);
        boundingRectPaint.setStrokeCap(Paint.Cap.BUTT);
        boundingRectPaint.setColor(boundingBoxColor);

        roiPaint = new Paint();
        roiPaint.setAntiAlias(true);
        roiPaint.setStrokeCap(Paint.Cap.BUTT);
        roiPaint.setColor(roiColor);

        contourPaint = new Paint();
        contourPaint.setStyle(Paint.Style.STROKE);
        contourPaint.setColor(contourColor);
    }


    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        lastFrame.set(createBitmap(width, height, Bitmap.Config.RGB_565));
        Log.i("cv test", "gotten to init of processor");
        frameWidth = width;
        frameHeight = height;

        roi = roiImg.asOpenCvRect(width, height);

        Core.setNumThreads(1);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Bitmap b = createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);

        roiMat_userColorSpace = frame.clone();
        //timer.reset();
        Imgproc.cvtColor(roiMat_userColorSpace, roiMat_userColorSpace, Imgproc.COLOR_RGB2HSV);
        //Log.i("cv test", "time to convert color: " + timer.milliseconds());
        if (colors.isEmpty())
            Core.inRange(roiMat_userColorSpace, colorRange.min, colorRange.max, mask);
        else {
            mask = new Mat();
            boolean first = true;
            for (ColorRange color : colors) {
                //Log.i("cv test", "color: " + color.toString());
                if (first) {
                    Core.inRange(roiMat_userColorSpace, color.min, color.max, mask);
                    first = false;
                } else {
                    Core.inRange(roiMat_userColorSpace, color.min, color.max, temp);
                    Core.bitwise_or(mask, temp, mask);
                }
            }
        }


        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        timer.reset();
        Imgproc.findContours(mask, contours, hierarchy, contourCode, Imgproc.RETR_LIST);
        //Log.i("cv test", "time to find contours: " + timer.milliseconds());
        hierarchy.release();

        ArrayList<Blob> blobs = new ArrayList<>();
        for (MatOfPoint contour : contours)
        {
            blobs.add(new BlobImpl(contour));
        }
        timer.reset();
        // Apply filters.
        synchronized (lockFilters)
        {
            for (BlobFilter filter : filters)
            {
                switch (filter.criteria)
                {
                    case BY_CONTOUR_AREA:
                        Util.filterByArea(filter.minValue, filter.maxValue, blobs);
                        break;
                    case BY_DENSITY:
                        Util.filterByDensity(filter.minValue, filter.maxValue, blobs);
                        break;
                    case BY_ASPECT_RATIO:
                        Util.filterByAspectRatio(filter.minValue, filter.maxValue, blobs);
                        break;
                }
            }
        }
        //Log.i("cv test", "time to filter contours: " + timer.milliseconds());
        timer.reset();
        // Apply sorting.
        BlobSort sort = this.sort; // Put the field into a local variable for thread safety.
        if (sort != null)
        {
            switch (sort.criteria)
            {
                case BY_CONTOUR_AREA:
                    Util.sortByArea(sort.sortOrder, blobs);
                    break;
                case BY_DENSITY:
                    Util.sortByDensity(sort.sortOrder, blobs);
                    break;
                case BY_ASPECT_RATIO:
                    Util.sortByAspectRatio(sort.sortOrder, blobs);
                    break;
            }
        }
        else
        {
            // Apply a default sort by area
            Util.sortByArea(SortOrder.DESCENDING, blobs);
        }
        //Log.i("cv test", "time to sort contours: " + timer.milliseconds());

        // Deep copy this to prevent concurrent modification exception
        userBlobs = new ArrayList<>(blobs);
        timer.reset();
        if (!blobs.isEmpty())
            circle = blobs.get(0).getCircle();
        else
            circle = new Circle(67000000,6700000,0);
        //Log.i("cv test", "time to get circle: " + timer.milliseconds());

        return blobs;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        if (Objects.isNull(userContext)) {
            return;
        }

        ArrayList<Blob> blobs = (ArrayList<Blob>) userContext;

        contourPaint.setStrokeWidth(scaleCanvasDensity * 4);
        boundingRectPaint.setStrokeWidth(scaleCanvasDensity * 10);
        roiPaint.setStrokeWidth(scaleCanvasDensity * 10);
        if (blobs.isEmpty()) {
            return;
        }
        if (circle != null)
            canvas.drawCircle(circle.getX() * scaleBmpPxToCanvasPx, circle.getY() * scaleBmpPxToCanvasPx, circle.getRadius() * scaleBmpPxToCanvasPx, contourPaint);
    }


    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx)
    {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void addFilter(BlobFilter filter)
    {
        synchronized (lockFilters)
        {
            filters.add(filter);
        }
    }

    @Override
    public void removeFilter(BlobFilter filter)
    {
        synchronized (lockFilters)
        {
            filters.remove(filter);
        }
    }

    @Override
    public void removeAllFilters()
    {
        synchronized (lockFilters)
        {
            filters.clear();
        }
    }

    @Override
    public void setSort(BlobSort sort)
    {
        this.sort = sort;
    }

    @Override
    public List<Blob> getBlobs()
    {
        return userBlobs;
    }

    public void addColors(ColorRange... colorRanges) {
        Collections.addAll(colors, colorRanges);
    }
    public void addColor(ColorRange color) {
        Log.i("cv test", "adding color: " + color.toString());
        colors.add(color);
        Log.i("cv test", "total colors: " + colors.size());
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer ->
                bitmapConsumer.accept(lastFrame.get())
        );
    }

    class BlobImpl extends Blob
    {
        private MatOfPoint contour;
        private Point[] contourPts;
        private MatOfPoint2f contourAsFloat;
        private int area = -1;
        private double density = -1;
        private double aspectRatio = -1;
        private RotatedRect rect;
        private double arcLength = -1;
        private double circularity = -1;
        private Circle circle;

        BlobImpl(MatOfPoint contour)
        {
            this.contour = contour;
        }

        @Override
        public MatOfPoint getContour()
        {
            return contour;
        }

        @Override
        public Point[] getContourPoints()
        {
            if (contourPts == null)
            {
                contourPts = contour.toArray();
            }

            return contourPts;
        }

        @Override
        public MatOfPoint2f getContourAsFloat() {
            if (contourAsFloat == null) {
                contourAsFloat = new MatOfPoint2f(getContourPoints());
            }

            return contourAsFloat;
        }

        @Override
        public int getContourArea()
        {
            if (area < 0)
            {
                area = Math.max(1, (int) Imgproc.contourArea(contour));  //  Fix zero area issue
            }

            return area;
        }

        @Override
        public double getDensity()
        {
            Point[] contourPts = getContourPoints();

            if (density < 0)
            {
                // Compute the convex hull of the contour
                MatOfInt hullMatOfInt = new MatOfInt();
                Imgproc.convexHull(contour, hullMatOfInt);

                // The convex hull calculation tells us the INDEX of the points which
                // which were passed in eariler which form the convex hull. That's all
                // well and good, but now we need filter out that original list to find
                // the actual POINTS which form the convex hull
                Point[] hullPoints = new Point[hullMatOfInt.rows()];
                List<Integer> hullContourIdxList = hullMatOfInt.toList();

                for (int i = 0; i < hullContourIdxList.size(); i++)
                {
                    hullPoints[i] = contourPts[hullContourIdxList.get(i)];
                }

                double hullArea = Math.max(1.0,Imgproc.contourArea(new MatOfPoint(hullPoints)));  //  Fix zero area issue

                density = getContourArea() / hullArea;
            }
            return density;
        }

        @Override
        public double getAspectRatio()
        {
            if (aspectRatio < 0)
            {
                RotatedRect r = getBoxFit();

                double longSize  = Math.max(1, Math.max(r.size.width, r.size.height));
                double shortSize = Math.max(1, Math.min(r.size.width, r.size.height));

                aspectRatio = longSize / shortSize;
            }

            return aspectRatio;
        }

        @Override
        public RotatedRect getBoxFit()
        {
            if (rect == null)
            {
                rect = Imgproc.minAreaRect(getContourAsFloat());
            }
            return rect;
        }

        @Override
        public double getArcLength()
        {
            if (arcLength < 0)
            {
                arcLength = Imgproc.arcLength(getContourAsFloat(), true);
            }

            return arcLength;
        }

        @Override
        public double getCircularity()
        {
            if (circularity < 0)
            {
                circularity = 4 * Math.PI * (getContourArea() / Math.pow(getArcLength(), 2));
            }

            return circularity;
        }

        @Override
        public Circle getCircle()
        {
            if (circle == null)
            {
                Point center = new Point();
                float[] radius = new float[1];
                Imgproc.minEnclosingCircle(getContourAsFloat(), center, radius);
                circle = new Circle(center, radius[0]);
            }

            return circle;
        }
    }
    public Circle getCircle() {
        return circle;
    }
}
