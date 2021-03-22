package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;
import android.util.Log;
import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.apache.commons.math3.distribution.NormalDistribution;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingStackDetector {

    public enum RingStackResult {
        ZERO(0),
        ONE(1),
        FOUR(4);

        public final int ringCount;

        RingStackResult(int ringCount) {
            this.ringCount = ringCount;
        }
    }

    private final OpenCvInternalCamera camera;
    private final RingDetectionPipeline pipeline = new RingDetectionPipeline();
    private volatile Pair<RingStackResult, Double> result = null;
    private volatile boolean saveImageNext = true;
    private Telemetry telemetry;

    public RingStackDetector(OpMode opMode, Telemetry telemetry) {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK,
                cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(320 * 3, 240 * 3, OpenCvCameraRotation.UPSIDE_DOWN);
        this.telemetry = telemetry;
    }

    public void saveImage() {
        saveImageNext = true;
    }

    public Optional<Pair<RingStackResult, Double>> currentlyDetected() {
        return Optional.ofNullable(result);
    }

    public void setFlashLight(boolean value) {
        camera.setFlashlightEnabled(value);
    }

    public void stop() {
        setFlashLight(false);
        camera.stopStreaming();
    }

    public void close() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    class RingDetectionPipeline extends OpenCvPipeline {

        final Scalar lowerRange = new Scalar(0, 20, 80);
        final Scalar upperRange = new Scalar(20, 200, 255);

        static final double ONE_RING_AREA = 8000, FOUR_RING_AREA = 13000;
        static final double ST_DEV = 10;
        NormalDistribution one_nd = new NormalDistribution(ONE_RING_AREA, ST_DEV);
        NormalDistribution four_nd = new NormalDistribution(FOUR_RING_AREA, ST_DEV);

        final Mat test = new Mat(),
                edgeDetector = new Mat(),
                smoothEdges = new Mat(),
                contourDetector = new Mat();
        final MatOfPoint2f polyDpResult = new MatOfPoint2f();
        final List<Rect> bounds = new ArrayList<>();
        final Size gaussianKernelSize = new Size(9, 9);

        @SuppressLint("SdCardPath")
        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, test, Imgproc.COLOR_RGB2HLS);
            Core.inRange(test, lowerRange, upperRange, edgeDetector);
            Imgproc.GaussianBlur(edgeDetector, smoothEdges, gaussianKernelSize, 0, 0);

            ArrayList<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(smoothEdges, contours, contourDetector,
                    Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            extractRectBounds(contours);

            for (Rect t : bounds) {
                Imgproc.rectangle(input, t, lowerRange, 2);
            }

            result = identifyStackFromBounds().orElse(null);
            if (saveImageNext) {
                Mat cvt = new Mat();
                Imgproc.cvtColor(input, cvt, Imgproc.COLOR_RGB2BGR);
                Log.i("RingStackDetector", "saving current pipeline image");
                for (Rect r : bounds) {
                    Log.i("RingStackDetector", String.format("result x=%d y=%d width=%d height=%d area=%.2f", r.x, r.y, r.width, r.height, r.area()));
                }
                Imgcodecs.imwrite("/sdcard/FIRST/pipe-img.png", cvt);
                Imgcodecs.imwrite("/sdcard/FIRST/pipe-img-smoothEdges.png", smoothEdges);
                saveImageNext = false;
                cvt.release();
            }
            return input;
        }

        private Optional<Pair<RingStackResult, Double>> identifyStackFromBounds() {
            if (bounds.size() == 0) {
                return Optional.of(Pair.create(RingStackResult.ZERO, 0.7));
            }
            double maxArea = bounds.stream().map(Rect::area).max(Comparator.naturalOrder()).get();
            if (Math.abs(maxArea - ONE_RING_AREA) < Math.abs(maxArea - FOUR_RING_AREA)) {
                return Optional.of(Pair.create(RingStackResult.ONE, 0.8));
            } else {
                return Optional.of(Pair.create(RingStackResult.FOUR, 0.8));
            }
        }

        private void extractRectBounds(ArrayList<MatOfPoint> contours) {
            bounds.clear();
            for (MatOfPoint contour : contours) {
                // if polydp fails, switch to a local new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), polyDpResult, 3, true);
                Rect r = Imgproc.boundingRect(new MatOfPoint(polyDpResult.toArray()));
                if (r.y > 300 && r.area() > ONE_RING_AREA / 2)
                    addCombineRectangle(bounds, r, bounds.size() - 1);
            }
        }

        private boolean overlaps(Rect a, Rect b) {
            return a.tl().inside(b) || a.br().inside(b) || b.tl().inside(a) || b.br().inside(a);
        }

        private Rect combineRect(Rect a, Rect b) {
            int topY = (int) Math.min(a.tl().y, b.tl().y);
            int leftX = (int) Math.min(a.tl().x, b.tl().x);
            int bottomY = (int) Math.max(a.br().y, b.br().y);
            int rightX = (int) Math.max(a.br().x, b.br().x);
            return new Rect(leftX, topY, rightX - leftX, bottomY - topY);
        }

        private void addCombineRectangle(List<Rect> list, Rect newRect, int ptr) {
            for (int i = ptr; i >= 0; i--) {
                Rect existing = list.get(i);
                if (overlaps(newRect, existing)) {
                    list.remove(i);
                    addCombineRectangle(list, combineRect(existing, newRect), i - 1);
                    return;
                }
            }
            list.add(newRect);
        }
    }
}
