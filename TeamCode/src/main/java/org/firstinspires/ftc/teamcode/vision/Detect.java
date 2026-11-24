package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class Detect implements VisionProcessor, CameraStreamSource {

    // true only for a camera mounted on its side (the phone videos); a Logitech webcam streams landscape
    public static boolean PORTRAIT = false;
    public static int WORK_WIDTH = 650;
    public static int THREADS = 4;
    // raise to 3-5 during matches to save CPU; 1 = every frame
    public static int STREAM_EVERY = 5;

    // ---- camera: Logitech C270 ----
    public static String webcamName = "Webcam 1"; // PLACEHOLDER: match the robot config
    // the SDK only has the C270's calibration at 640x480 (or another 4:3 size, scaled): keep 640x480
    public static int frameWidth = 640, frameHeight = 480;
    // <= 0 leaves the camera on auto. Auto white balance drifts the ball/tile colours this thresholds on, so lock it on
    // the robot once tuned (e.g. 4600 K); manual exposure (e.g. 15 ms) also cuts motion blur.
    public static int whiteBalanceK = 0, exposureMs = 0, gain = 0;
    // ---- camera mount, relative to the robot centre (inches, degrees). The only copy: Pickup/Impose read these too ----
    public static double camForwardIn = 0, camLeftIn = 0; // PLACEHOLDER: measure on robot
    public static double camHeightIn = 4; // PLACEHOLDER: measure on robot
    public static double camPitchDownDeg = 0, camYawLeftDeg = 0; // PLACEHOLDER: measure on robot (0 = straight ahead)

    public static int tileR = 128, tileG = 128, tileB = 128; // equal = no white-balance correction (C270 colours are already neutral)
    public static int minHue = 70, maxHue = 118, minChroma = 40, minBright = 61, maxBright = 255;
    public static int holeFillMaxPx = 24;
    public static int blemishMaxPx = 12, blemishDarkness = 15, smoothSize = 15;
    public static int singleMinAreaPx = 60;
    public static double fitDistancePctg = 0.1, fitDistanceMinPx = 1, fitMinPctg = 0.7;
    public static double valleyReachPctg = 0.45;
    public static int valleyFallbackReachPx = 2;
    public static double valleyDepthPctg = 8, valleyNoisePctg = 20, connectInsidePctg = 0.6, guessEndSkipPctg = 0.25;
    public static int arcMinPoints = 8;
    public static double outlineMinPctg = 0.25, outlineFitMaxPctg = 0.3, arcShiftPctg = 0.3, arcShiftFullDeg = 90, arcSnapPctg = 0.3, arcMeetPctg = 0.1, arcBendPctg = 0.3;
    public static double sizeVoteSmallestPx = 3, sizeVoteStepPctg = 0.12, sizeVoteMinCoverPctg = 0.45, sizeVoteValleyWeight = 0.5, sizeSampleMinPx = 4.5;
    public static double seedVoteBlurPctg = 0.2, seedMinCoverPctg = 0.03, seedSpacingPctg = 0.5, seedMinFillPctg = 0.4, seedClimbBlurPx = 1.5, seedSameSpotReachPctg = 0.7;
    public static double ballRadiusCm = 3.55;
    static final double CM = 2.54; // cm per inch
    public static double sizeFilterPctg = 0.5, ballMinRadiusPx = 2.0, ballMaxRadiusPx = 120.0, fieldMaxDistanceCm = 450;
    public static double remainingOpenPctg = 0.25, remainingMinBallPctg = 0.4, remainingBallAreaPctg = 1.0, remainingMinRadiusPx = 2.5;
    public static int regionMinPx = 15;
    public static double seamWeight = 0.3;
    public static int edgeMinPoints = 12, fitRefineRounds = 3;
    public static double radiusTolerancePctg = 0.25, duplicateCenterPctg = 0.3, circleMinFillPctg = 0.4, radiusRejectPctg = 0.5;
    public static int refitRounds = 10;

    static int H, W;
    static long cvNanos;
    // Fallback intrinsics (the phone the videos were shot on). init() replaces them with the FTC SDK's built-in calibration
    // for the webcam at the streamed resolution (the Logitech C270 has one at 640x480 only). Read by Pickup for Impose.
    public static double[] calK = {477.86395397156593, 0.0, 457.1946537778554, 0.0, 479.2195587816585, 641.119897833739, 0.0, 0.0, 1.0};
    public static double[] calD = {0.008500997918842705, -0.02061464151378009, 0.0007540416777619344, -0.00012448636069353983, 0.0};
    public static double calW = 960.0, calH = 1280.0;
    public static volatile boolean calFromSdk = false;
    // size of the image Ball.x / Ball.y refer to (transposed when PORTRAIT); 0 until the first frame
    public static volatile int frameW, frameH;

    static long tic() { return System.nanoTime(); }
    static void toc(long start) { cvNanos += System.nanoTime() - start; }

    interface Expect { double at(double row); }

    static final class Box {
        int top, bottom, left, right, label;
        Box(int top, int bottom, int left, int right, int label) { this.top = top; this.bottom = bottom; this.left = left; this.right = right; this.label = label; }
    }

    static final class Found { double x, y, r, expected = -1; double[] position, raw; boolean kept = true, estimated; }

    static final class Fit {
        double cx, cy, r, score, mx, my;
        double[] px, py;
        int area, outer, label;
        boolean fixed, drop;
    }

    static final class Conn {
        double sx, sy, ex, ey, pointX, pointY;
        boolean guessed, hasArc, frontIsEnd, hasSpan;
        double acx, acy, ar, span0, span1;
        double[] nx, ny;
    }

    static int[] insideIdx = new int[0];
    static int insideCount;
    static int insideTop, insideBottom, insideLeft, insideRight;
    static int[] pieceBuf = new int[0], queueBuf = new int[0];
    static byte[] lastL;
    static byte[] aBgrBuf, aRemBuf;
    static Mat aRemMat;

    static void indexInside(boolean[] inside) {
        if (insideIdx.length < H * W) insideIdx = new int[H * W];
        int n = 0, top = H, bottom = -1, left = W, right = -1;
        for (int y = 0; y < H; y++) {
            int row = y * W;
            for (int x = 0; x < W; x++) {
                if (!inside[row + x]) continue;
                insideIdx[n++] = row + x;
                if (y < top) top = y;
                bottom = y;
                if (x < left) left = x;
                if (x > right) right = x;
            }
        }
        insideCount = n;
        insideTop = top; insideBottom = bottom; insideLeft = left; insideRight = right;
    }

    static Box insideBox(int margin) {
        if (insideCount == 0) return null;
        return new Box(Math.max(insideTop - margin, 0), Math.min(insideBottom + 1 + margin, H), Math.max(insideLeft - margin, 0), Math.min(insideRight + 1 + margin, W), 0);
    }

    static boolean isOuter(boolean[] inside, int i) {
        int y = i / W, x = i - y * W;
        return !inside[i] || y == 0 || x == 0 || y == H - 1 || x == W - 1 || !inside[i - W] || !inside[i + W] || !inside[i - 1] || !inside[i + 1];
    }

    static byte[] getBytes(Mat m) { byte[] d = new byte[(int) (m.total() * m.channels())]; m.get(0, 0, d); return d; }
    static float[] getFloats(Mat m) { float[] d = new float[(int) (m.total() * m.channels())]; m.get(0, 0, d); return d; }
    static int[] getInts(Mat m) { int[] d = new int[(int) (m.total() * m.channels())]; m.get(0, 0, d); return d; }
    static Mat bytesMat(byte[] d, int rows, int cols) { Mat m = tmp(rows, cols, CvType.CV_8UC1); m.put(0, 0, d); return m; }
    static Mat floatsMat(float[] d, int rows, int cols) { Mat m = tmp(rows, cols, CvType.CV_32FC1); m.put(0, 0, d); return m; }

    static byte[] crop(byte[] a, Box b, int channels) {
        int w = b.right - b.left;
        byte[] out = new byte[(b.bottom - b.top) * w * channels];
        for (int y = b.top; y < b.bottom; y++) System.arraycopy(a, (y * W + b.left) * channels, out, (y - b.top) * w * channels, w * channels);
        return out;
    }

    // ---- group A helpers (non-allocating variants, scratch buffers) ----
    static byte[] aBuf(byte[] cur, int n) { return cur != null && cur.length == n ? cur : new byte[n]; }
    static byte[] aGrow(byte[] cur, int n) { return cur.length >= n ? cur : new byte[(n + n / 4 + 11) / 12 * 12]; }
    static int[] aGrowI(int[] cur, int n) { return cur.length >= n ? cur : new int[(n + n / 4 + 11) / 12 * 12]; }
    static Mat aMat(Mat cur, int rows, int cols, int type) {
        if (cur != null && cur.rows() == rows && cur.cols() == cols && cur.type() == type) return cur;
        if (cur != null) cur.release();
        return new Mat(rows, cols, type);
    }
    static void aGetBytesInto(Mat m, byte[] d) { m.get(0, 0, d); }
    static void aCropInto(byte[] a, Box b, int channels, byte[] out) {
        int w = b.right - b.left;
        for (int y = b.top; y < b.bottom; y++) System.arraycopy(a, (y * W + b.left) * channels, out, (y - b.top) * w * channels, w * channels);
    }

    /** Per-thread scratch for the box jobs (runAll uses a pool). */
    static final class AScratch {
        byte[] b1 = new byte[0], b2 = new byte[0], b3 = new byte[0], b4 = new byte[0];
        int[] i1 = new int[0], i2 = new int[0];
        Mat m1 = new Mat(), m2 = new Mat(), m3 = new Mat(), m4 = new Mat(), m5 = new Mat();
    }
    static final ThreadLocal<AScratch> aScratch = ThreadLocal.withInitial(AScratch::new);

    static float[] cropF(float[] a, Box b) {
        int w = b.right - b.left;
        float[] out = new float[(b.bottom - b.top) * w];
        for (int y = b.top; y < b.bottom; y++) System.arraycopy(a, y * W + b.left, out, (y - b.top) * w, w);
        return out;
    }

    static double median(double[] values, int n) {
        if (n == 0) return Double.NaN;
        double[] c = Arrays.copyOf(values, n);
        Arrays.sort(c);
        return n % 2 == 1 ? c[n / 2] : (c[n / 2 - 1] + c[n / 2]) / 2;
    }

    static double[] lstsq(double[][] a, double[] b) {
        int m = a.length, n = a[0].length;
        double[][] q = new double[m][];
        for (int i = 0; i < m; i++) q[i] = a[i].clone();
        double[] y = b.clone();
        double[] diag = new double[n];
        for (int k = 0; k < n; k++) {
            double norm = 0;
            for (int i = k; i < m; i++) norm += q[i][k] * q[i][k];
            norm = Math.sqrt(norm);
            if (norm == 0) { diag[k] = 0; continue; }
            double alpha = q[k][k] > 0 ? -norm : norm;
            double[] v = new double[m];
            for (int i = k; i < m; i++) v[i] = q[i][k];
            v[k] -= alpha;
            double vv = 0;
            for (int i = k; i < m; i++) vv += v[i] * v[i];
            if (vv == 0) { diag[k] = alpha; continue; }
            for (int j = k; j < n; j++) {
                double dot = 0;
                for (int i = k; i < m; i++) dot += v[i] * q[i][j];
                double f = 2 * dot / vv;
                for (int i = k; i < m; i++) q[i][j] -= f * v[i];
            }
            double dot = 0;
            for (int i = k; i < m; i++) dot += v[i] * y[i];
            double f = 2 * dot / vv;
            for (int i = k; i < m; i++) y[i] -= f * v[i];
            diag[k] = q[k][k];
        }
        double[] x = new double[n];
        for (int k = n - 1; k >= 0; k--) {
            double s = y[k];
            for (int j = k + 1; j < n; j++) s -= q[k][j] * x[j];
            x[k] = Math.abs(diag[k]) < 1e-12 ? 0 : s / q[k][k];
        }
        return x;
    }

    static final class CScratch { double[] q = new double[0], y = new double[0], v = new double[0]; boolean[] keep = new boolean[0]; }
    static final ThreadLocal<CScratch> cScratch = ThreadLocal.withInitial(CScratch::new);

    // Same Householder QR and operation order as lstsq(), on a flat row-major m x n matrix (modified in place) and rhs y.
    static double[] cLstsqFlat(double[] q, double[] y, double[] v, int m, int n) {
        double[] diag = new double[n];
        for (int k = 0; k < n; k++) {
            double norm = 0;
            for (int i = k; i < m; i++) norm += q[i * n + k] * q[i * n + k];
            norm = Math.sqrt(norm);
            if (norm == 0) { diag[k] = 0; continue; }
            double alpha = q[k * n + k] > 0 ? -norm : norm;
            for (int i = k; i < m; i++) v[i] = q[i * n + k];
            v[k] -= alpha;
            double vv = 0;
            for (int i = k; i < m; i++) vv += v[i] * v[i];
            if (vv == 0) { diag[k] = alpha; continue; }
            for (int j = k; j < n; j++) {
                double dot = 0;
                for (int i = k; i < m; i++) dot += v[i] * q[i * n + j];
                double f = 2 * dot / vv;
                for (int i = k; i < m; i++) q[i * n + j] -= f * v[i];
            }
            double dot = 0;
            for (int i = k; i < m; i++) dot += v[i] * y[i];
            double f = 2 * dot / vv;
            for (int i = k; i < m; i++) y[i] -= f * v[i];
            diag[k] = q[k * n + k];
        }
        double[] x = new double[n];
        for (int k = n - 1; k >= 0; k--) {
            double s = y[k];
            for (int j = k + 1; j < n; j++) s -= q[k * n + j] * x[j];
            x[k] = Math.abs(diag[k]) < 1e-12 ? 0 : s / q[k * n + k];
        }
        return x;
    }

    static double[] polyfitLine(double[] xs, double[] ys, int n) {
        double mx = 0, my = 0;
        for (int i = 0; i < n; i++) { mx += xs[i]; my += ys[i]; }
        mx /= n; my /= n;
        double sxx = 0, sxy = 0;
        for (int i = 0; i < n; i++) { sxx += (xs[i] - mx) * (xs[i] - mx); sxy += (xs[i] - mx) * (ys[i] - my); }
        double slope = sxy / sxx;
        return new double[]{slope, my - slope * mx};
    }

    static Expect radiusByRow(List<double[]> refs) {
        if (refs.size() < 2) return null;
        int n = refs.size();
        double[] rows = new double[n], radii = new double[n];
        double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
        for (int i = 0; i < n; i++) { rows[i] = refs.get(i)[0]; radii[i] = refs.get(i)[1]; lo = Math.min(lo, rows[i]); hi = Math.max(hi, rows[i]); }
        if (hi - lo < 1) { double med = median(radii, n); return row -> med; }
        double[] line = polyfitLine(rows, radii, n);
        return row -> Math.max(line[0] * row + line[1], 1.0);
    }

    static Expect trimmedRadiusLine(List<Double> rowList, List<Double> radiiList) {
        int n = rowList.size();
        double[] rows = new double[n], radii = new double[n];
        double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
        for (int i = 0; i < n; i++) { rows[i] = rowList.get(i); radii[i] = radiiList.get(i); lo = Math.min(lo, rows[i]); hi = Math.max(hi, rows[i]); }
        double slope = 0;
        if (n >= 2 && hi - lo >= 1) {
            double[] slopes = new double[n * (n - 1) / 2];
            int c = 0;
            for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++) { double spread = rows[j] - rows[i]; if (Math.abs(spread) >= 1) slopes[c++] = (radii[j] - radii[i]) / spread; }
            slope = median(slopes, c);
        }
        double[] rest = new double[n];
        for (int i = 0; i < n; i++) rest[i] = radii[i] - slope * rows[i];
        double intercept = median(rest, n);
        double[] error = new double[n];
        for (int i = 0; i < n; i++) error[i] = Math.abs(slope * rows[i] + intercept - radii[i]);
        double limit = Math.max(2.5 * median(error, n), 0.5);
        double[] kr = new double[n], kd = new double[n];
        int k = 0;
        double klo = Double.MAX_VALUE, khi = -Double.MAX_VALUE;
        for (int i = 0; i < n; i++) if (error[i] <= limit) { kr[k] = rows[i]; kd[k] = radii[i]; k++; klo = Math.min(klo, rows[i]); khi = Math.max(khi, rows[i]); }
        if (k >= 3 && khi - klo >= 1) { double[] line = polyfitLine(kr, kd, k); slope = line[0]; intercept = line[1]; }
        double s = slope, b = intercept;
        return row -> Math.max(s * row + b, 1.5);
    }

    static double tolerance(double r) { return Math.max(fitDistancePctg * r, fitDistanceMinPx); }

    static double[] fixedRadiusCenter(double[] xs, double[] ys, boolean[] keep, double cx, double cy, double r) {
        int count = 0;
        for (int i = 0; i < xs.length; i++) if (keep == null || keep[i]) count++;
        for (int it = 0; it < 20; it++) {
            double sx = 0, sy = 0;
            for (int i = 0; i < xs.length; i++) {
                if (keep != null && !keep[i]) continue;
                double ax = xs[i] - cx, ay = ys[i] - cy;
                double len = Math.max(dist(ax, ay), 1e-6);
                sx += xs[i] - r * ax / len;
                sy += ys[i] - r * ay / len;
            }
            cx = sx / count;
            cy = sy / count;
        }
        return new double[]{cx, cy};
    }

    static int markOnCircle(double[] xs, double[] ys, double cx, double cy, double r, boolean[] keep) {
        double tol = tolerance(r);
        int count = 0;
        for (int i = 0; i < xs.length; i++) { keep[i] = Math.abs(dist(xs[i] - cx, ys[i] - cy) - r) <= tol; if (keep[i]) count++; }
        return count;
    }

    static double[] weightedCircle(double[] xs, double[] ys, double[] w, boolean[] keep) {
        int count = 0;
        for (int i = 0; i < xs.length; i++) if (keep == null || keep[i]) count++;
        CScratch sc = cScratch.get();
        if (sc.q.length < count * 3) sc.q = new double[count * 3];
        if (sc.y.length < count) { sc.y = new double[count]; sc.v = new double[count]; }
        double[] a = sc.q, b = sc.y;
        int row = 0;
        for (int i = 0; i < xs.length; i++) {
            if (keep != null && !keep[i]) continue;
            double root = w == null ? 1 : Math.sqrt(w[i]);
            a[row * 3] = xs[i] * root; a[row * 3 + 1] = ys[i] * root; a[row * 3 + 2] = root;
            b[row] = (xs[i] * xs[i] + ys[i] * ys[i]) * root;
            row++;
        }
        double[] s = cLstsqFlat(a, b, sc.v, count, 3);
        double cx = s[0] / 2, cy = s[1] / 2;
        double r2 = s[2] + cx * cx + cy * cy;
        if (r2 <= 0) return null;
        return new double[]{cx, cy, Math.sqrt(r2)};
    }

    static double[] fitRegion(double[] xs, double[] ys, double[] w) {
        int n = xs.length;
        boolean[] keep = new boolean[n];
        double[] c = weightedCircle(xs, ys, w, null);
        if (c == null) return null;
        for (int it = 0; it < fitRefineRounds; it++) {
            if (markOnCircle(xs, ys, c[0], c[1], c[2], keep) < Math.max(5, 0.3 * n)) break;
            double[] refit = weightedCircle(xs, ys, w, keep);
            if (refit == null) break;
            c = refit;
        }
        double score = (double) markOnCircle(xs, ys, c[0], c[1], c[2], keep) / n;
        return new double[]{c[0], c[1], c[2], score};
    }

    static double circleFill(byte[] filled, double cx, double cy, double r) {
        int left = Math.max((int) Math.floor(cx - r), 0), right = Math.min((int) Math.ceil(cx + r) + 1, W);
        int top = Math.max((int) Math.floor(cy - r), 0), bottom = Math.min((int) Math.ceil(cy + r) + 1, H);
        if (left >= right || top >= bottom) return 0;
        int count = 0;
        final double rr = r * r;
        for (int y = top; y < bottom; y++) {
            double down = (y - cy) * (y - cy);
            int row = y * W;
            for (int x = left; x < right; x++) if ((x - cx) * (x - cx) + down <= rr && filled[row + x] != 0) count++;
        }
        return count / Math.max(Math.PI * r * r, 1.0);
    }

    static Box paddedBox(byte[] region, int margin) {
        int top = H, bottom = -1, left = W, right = -1;
        for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) if (region[y * W + x] != 0) { if (y < top) top = y; if (y > bottom) bottom = y; if (x < left) left = x; if (x > right) right = x; }
        if (bottom < 0) return null;
        return new Box(Math.max(top - margin, 0), Math.min(bottom + 1 + margin, H), Math.max(left - margin, 0), Math.min(right + 1 + margin, W), 0);
    }

    static int[] lastGroups;

    static int[] groupBuf = new int[0];

    static Mat aPbIn, aPbGrown, aPbLabels, aPbStats, aPbCent, aCnIn, aCnLabels;
    static final Map<Integer, Mat> aKernels = new HashMap<>();

    static Mat aKernel(int size) {
        Mat k = aKernels.get(size);
        if (k == null) { k = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(size, size)); aKernels.put(size, k); }
        return k;
    }

    static byte[] aPbCrop = new byte[0];
    static int[] aPbRoiLabels = new int[0];

    static List<Box> paddedBoxes(byte[] region, int margin) {
        int top = H, bottom = -1, left = W, right = W;
        right = -1;
        for (int y = 0; y < H; y++) {
            int row = y * W;
            int first = -1, last = -1;
            for (int x = 0; x < W; x++) if (region[row + x] != 0) { first = x; break; }
            if (first < 0) continue;
            for (int x = W - 1; x > first; x--) if (region[row + x] != 0) { last = x; break; }
            if (last < 0) last = first;
            if (y < top) top = y;
            bottom = y;
            if (first < left) left = first;
            if (last > right) right = last;
        }
        if (groupBuf.length < H * W) groupBuf = new int[H * W];
        Arrays.fill(groupBuf, 0, H * W, 0);
        lastGroups = groupBuf;
        List<Box> boxes = new ArrayList<>();
        if (bottom < 0) return boxes;
        // ROI: bounding box grown by the margin, origin kept even so the labelling order is unchanged
        int r0 = Math.max(top - margin, 0) & ~1, c0 = Math.max(left - margin, 0) & ~1;
        int r1 = Math.min(bottom + 1 + margin, H), c1 = Math.min(right + 1 + margin, W);
        int rh = r1 - r0, rw = c1 - c0;
        aPbCrop = aGrow(aPbCrop, rh * rw);
        for (int y = 0; y < rh; y++) System.arraycopy(region, (y + r0) * W + c0, aPbCrop, y * rw, rw);
        aPbIn = aMat(aPbIn, rh, rw, CvType.CV_8UC1);
        aPbIn.put(0, 0, aPbCrop);
        if (aPbGrown == null) { aPbGrown = new Mat(); aPbLabels = new Mat(); aPbStats = new Mat(); aPbCent = new Mat(); }
        long s = tic();
        Imgproc.dilate(aPbIn, aPbGrown, aKernel(2 * margin + 1));
        int count = Imgproc.connectedComponentsWithStats(aPbGrown, aPbLabels, aPbStats, aPbCent, 8, CvType.CV_32S);
        toc(s);
        aPbRoiLabels = aGrowI(aPbRoiLabels, rh * rw);
        aPbLabels.get(0, 0, aPbRoiLabels);
        for (int y = 0; y < rh; y++) System.arraycopy(aPbRoiLabels, y * rw, groupBuf, (y + r0) * W + c0, rw);
        int[] st = getInts(aPbStats);
        for (int label = 1; label < count; label++) {
            int l = st[label * 5] + c0, t = st[label * 5 + 1] + r0, w = st[label * 5 + 2], h = st[label * 5 + 3];
            boxes.add(new Box(t, t + h, l, l + w, label));
        }
        return boxes;
    }

    static int[][] labelBufs = new int[4][0];

    static int[] connected(byte[] mask, int connectivity, int[] countOut, int slot) {
        aCnIn = aMat(aCnIn, H, W, CvType.CV_8UC1);
        aCnIn.put(0, 0, mask);
        if (aCnLabels == null) aCnLabels = new Mat();
        long s = tic();
        countOut[0] = Imgproc.connectedComponents(aCnIn, aCnLabels, connectivity, CvType.CV_32S);
        toc(s);
        if (labelBufs[slot].length != H * W) labelBufs[slot] = new int[H * W];
        aCnLabels.get(0, 0, labelBufs[slot]);
        return labelBufs[slot];
    }

    static Mat aLutScaled, aWbOut;
    static int aLutR = -1, aLutG = -1, aLutB = -1;

    static Mat whiteBalance(Mat image) {
        if (aLutScaled == null || aLutR != tileR || aLutG != tileG || aLutB != tileB) {
            double target = (tileR + tileG + tileB) / 3.0;
            Mat lut = new Mat(1, 256, CvType.CV_8UC3);
            byte[] values = new byte[256 * 3];
            for (int i = 0; i < 256; i++) for (int c = 0; c < 3; c++) values[i * 3 + c] = (byte) i;
            lut.put(0, 0, values);
            Mat scaled = new Mat();
            Core.multiply(lut, new Scalar(target / tileB, target / tileG, target / tileR, 0), scaled);
            lut.release();
            if (aLutScaled != null) aLutScaled.release();
            aLutScaled = scaled;
            aLutR = tileR; aLutG = tileG; aLutB = tileB;
        }
        if (aWbOut == null) aWbOut = new Mat();
        long s = tic();
        Core.LUT(image, aLutScaled, aWbOut);
        toc(s);
        return aWbOut;
    }

    static byte[] colorTable;

    static byte[] chromaTable = new byte[65536]; // tuning aid: chroma (capped 255) when the hue is in range, else 0

    static byte[] makeColorTable() {
        byte[] t = new byte[65536];
        for (int a = 0; a < 256; a++) for (int b = 0; b < 256; b++) {
            float fa = a - 128, fb = b - 128;
            float hue = (float) Math.toDegrees(Math.atan2(fb, fa));
            hue = hue % 360f;
            if (hue < 0) hue += 360f;
            float chroma = (float) dist(fa, fb);
            t[a * 256 + b] = (byte) (hue >= minHue && hue <= maxHue && chroma >= minChroma ? 255 : 0);
            chromaTable[a * 256 + b] = (byte) (hue >= minHue && hue <= maxHue ? Math.min(255, Math.round(chroma)) : 0);
        }
        return t;
    }

    static Mat aLab, aMkIn, aMkFilled, aHoleIn, aHoleOut, aBmIn, aBmOut, aAroundIn, aAroundOut, aPixIn, aPixOut;
    static byte[] aLabBuf, aMaskBuf, aLightBuf, aFilledBuf, aDiffBuf, aHolesBuf, aDarkBuf, aOutliersBuf, aMarkedBuf, aResultBuf, aRepaired, aAround, aSmoothLBuf, aPixBuf, aPixLabBuf;
    static Mat aRing;

    static byte[] yellowMask(Mat image) {
        int total = H * W;
        aLab = aMat(aLab, image.rows(), image.cols(), CvType.CV_8UC3);
        long s = tic();
        Imgproc.cvtColor(image, aLab, Imgproc.COLOR_BGR2Lab);
        toc(s);
        aLabBuf = aBuf(aLabBuf, total * 3);
        aGetBytesInto(aLab, aLabBuf);
        aMaskBuf = aBuf(aMaskBuf, total);
        aLightBuf = aBuf(aLightBuf, total);
        byte[] all = aLabBuf, mask = aMaskBuf, lightness = aLightBuf, table = colorTable;
        int on = 0;
        long sumL = 0;
        int[] hist = new int[256];
        for (int i = 0; i < total; i++) {
            int l = all[i * 3] & 255, a = all[i * 3 + 1] & 255, b = all[i * 3 + 2] & 255;
            sumL += l;
            lightness[i] = all[i * 3];
            mask[i] = (byte) (table[a * 256 + b] != 0 && l >= minBright && l <= maxBright ? 255 : 0);
            if (mask[i] != 0) on++;
            if (l >= minBright && l <= maxBright) hist[chromaTable[a * 256 + b] & 255]++;
        }
        StringBuilder h = new StringBuilder();
        for (int c = 40, sum = 0; c >= 10; c -= 5) {
            for (int k = c; k < (c == 40 ? 256 : c + 5); k++) sum += hist[k];
            h.insert(0, String.format(Locale.US, " >=%d: %.1f%%", c, 100.0 * sum / total));
        }
        debugChroma = h.toString().trim();
        lastL = lightness;
        debugMaskPct = 100.0 * on / total;
        // tuning aid: the colour the thresholds see (after white balance) in a 9x9 patch at the image centre
        double sl = 0, sa = 0, sb = 0;
        int n = 0;
        for (int y = H / 2 - 4; y <= H / 2 + 4; y++) for (int x = W / 2 - 4; x <= W / 2 + 4; x++) {
            int i = (y * W + x) * 3;
            sl += all[i] & 255; sa += (all[i + 1] & 255) - 128; sb += (all[i + 2] & 255) - 128; n++;
        }
        sa /= n; sb /= n;
        double hue = Math.toDegrees(Math.atan2(sb, sa));
        if (hue < 0) hue += 360;
        debugCenter = String.format(Locale.US, "L %.0f (min %d), hue %.0f (%d-%d), chroma %.0f (min %d); whole image L %.0f",
                sl / n, minBright, hue, minHue, maxHue, Math.hypot(sa, sb), minChroma, (double) sumL / total);
        return mask;
    }

    static byte[] fillHoles(byte[] mask) {
        int total = H * W;
        aMkIn = aMat(aMkIn, H, W, CvType.CV_8UC1);
        aMkIn.put(0, 0, mask);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = tmp();
        long s = tic();
        findContoursTracked(aMkIn, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
        toc(s);
        aFilledBuf = aBuf(aFilledBuf, total);
        byte[] out = aFilledBuf;
        System.arraycopy(mask, 0, out, 0, total);
        if (!contours.isEmpty()) {
            int[] h = getInts(hierarchy);
            boolean[] small = null;
            for (int i = 0; i < contours.size(); i++) {
                s = tic();
                boolean sm = h[i * 4 + 3] != -1 && Imgproc.contourArea(contours.get(i)) <= holeFillMaxPx;
                toc(s);
                if (sm) { if (small == null) small = new boolean[contours.size()]; small[i] = true; }
            }
            if (small != null) {
                aMkFilled = aMat(aMkFilled, H, W, CvType.CV_8UC1);
                aMkIn.copyTo(aMkFilled);
                s = tic();
                for (int i = 0; i < contours.size(); i++) if (small[i]) Imgproc.drawContours(aMkFilled, contours, i, new Scalar(255), -1);
                toc(s);
                aGetBytesInto(aMkFilled, out);
            }
        }
        return out;
    }

    static boolean aAnyBlemish;

    static byte[] findBlemishes(byte[] brightness, byte[] mask, byte[] filled) {
        final int total = H * W;
        if (aRing == null) aRing = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 3));
        Mat ring = aRing;
        aDiffBuf = aBuf(aDiffBuf, total);
        aHolesBuf = aBuf(aHolesBuf, total);
        byte[] diff = aDiffBuf, holes = aHolesBuf;
        boolean anyDiff = false;
        for (int i = 0; i < total; i++) {
            int d = Math.max((filled[i] & 255) - (mask[i] & 255), 0);
            diff[i] = (byte) d;
            if (d != 0) anyDiff = true;
        }
        if (anyDiff) {
            aHoleIn = aMat(aHoleIn, H, W, CvType.CV_8UC1);
            aHoleIn.put(0, 0, diff);
            if (aHoleOut == null) aHoleOut = new Mat();
            long s = tic();
            Imgproc.dilate(aHoleIn, aHoleOut, ring);
            toc(s);
            aGetBytesInto(aHoleOut, holes);
            for (int i = 0; i < total; i++) holes[i] = (byte) (holes[i] & filled[i]);
        } else {
            Arrays.fill(holes, (byte) 0);
        }
        int window = Math.min(Math.max(2 * blemishMaxPx + 1, 3), 255);
        aDarkBuf = aBuf(aDarkBuf, total);
        final byte[] dark = aDarkBuf;
        Arrays.fill(dark, (byte) 0);
        List<Box> boxes = paddedBoxes(mask, window / 2);
        int[] groups = lastGroups;
        List<Runnable> jobs = new ArrayList<>();
        final int fwindow = window;
        for (final Box b : boxes) jobs.add(() -> {
            int bw = b.right - b.left, bh = b.bottom - b.top, n = bw * bh;
            AScratch sc = aScratch.get();
            sc.b1 = aGrow(sc.b1, n);
            sc.b2 = aGrow(sc.b2, n);
            aCropInto(brightness, b, 1, sc.b1);
            sc.m1.create(bh, bw, CvType.CV_8UC1);
            sc.m1.put(0, 0, sc.b1);
            Imgproc.medianBlur(sc.m1, sc.m2, fwindow);
            sc.m2.get(0, 0, sc.b2);
            byte[] md = sc.b2;
            for (int y = 0; y < bh; y++) for (int x = 0; x < bw; x++) {
                int i = (y + b.top) * W + x + b.left;
                if (groups[i] == b.label && mask[i] != 0 && Math.max((md[y * bw + x] & 255) - (brightness[i] & 255), 0) > blemishDarkness) dark[i] = (byte) 255;
            }
        });
        runAll(jobs);
        aOutliersBuf = aBuf(aOutliersBuf, total);
        byte[] outliers = aOutliersBuf;
        boolean any = false;
        for (int i = 0; i < total; i++) { byte v = (byte) (dark[i] | holes[i]); outliers[i] = v; if (v != 0) any = true; }
        aAnyBlemish = false;
        if (!any) return outliers;
        int[] cnt = new int[1];
        int[] labels = connected(outliers, 8, cnt, 0);
        int count = cnt[0];
        double[] n = new double[count], sx = new double[count], sy = new double[count], sxx = new double[count], syy = new double[count], sxy = new double[count];
        for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) {
            int i = y * W + x;
            if (outliers[i] == 0) continue;
            int g = labels[i];
            n[g]++; sx[g] += x; sy[g] += y; sxx[g] += (double) x * x; syy[g] += (double) y * y; sxy[g] += (double) x * y;
        }
        boolean[] blemish = new boolean[count];
        boolean anyMarked = false;
        for (int g = 1; g < count; g++) {
            double nn = Math.max(n[g], 1);
            double mx = sx[g] / nn, my = sy[g] / nn;
            double vx = sxx[g] / nn - mx * mx + 1.0 / 12, vy = syy[g] / nn - my * my + 1.0 / 12, cov = sxy[g] / nn - mx * my;
            double longest = 4 * Math.sqrt(Math.max((vx + vy) / 2 + Math.sqrt(((vx - vy) / 2) * ((vx - vy) / 2) + cov * cov), 0));
            blemish[g] = longest <= blemishMaxPx;
            if (blemish[g]) anyMarked = true;
        }
        aResultBuf = aBuf(aResultBuf, total);
        byte[] result = aResultBuf;
        if (!anyMarked) { Arrays.fill(result, (byte) 0); return result; }
        aMarkedBuf = aBuf(aMarkedBuf, total);
        byte[] marked = aMarkedBuf;
        for (int i = 0; i < total; i++) marked[i] = (byte) (blemish[labels[i]] ? 255 : 0);
        aBmIn = aMat(aBmIn, H, W, CvType.CV_8UC1);
        aBmIn.put(0, 0, marked);
        if (aBmOut == null) aBmOut = new Mat();
        long s = tic();
        Imgproc.dilate(aBmIn, aBmOut, ring);
        toc(s);
        aGetBytesInto(aBmOut, result);
        boolean anyResult = false;
        for (int i = 0; i < total; i++) { byte v = (byte) (result[i] & filled[i]); result[i] = v; if (v != 0) anyResult = true; }
        aAnyBlemish = anyResult;
        return result;
    }

    static int[] aChanged = new int[0];
    static int aChangedCount;

    static byte[] smoothInside(byte[] bgr, byte[] brightness, byte[] mask, byte[] filled, byte[][] blemishOut) {
        byte[] blemishes = findBlemishes(brightness, mask, filled);
        blemishOut[0] = blemishes;
        final int total = H * W;
        aRepaired = aBuf(aRepaired, total * 3);
        final byte[] repaired = aRepaired;
        System.arraycopy(bgr, 0, repaired, 0, total * 3);
        aChangedCount = 0;
        findInpaint();
        if (!aAnyBlemish) return repaired;
        List<Box> boxes = paddedBoxes(blemishes, 9);
        int[] groups = lastGroups;
        List<Runnable> jobs = new ArrayList<>();
        for (final Box b : boxes) jobs.add(() -> {
            int bw = b.right - b.left, bh = b.bottom - b.top, n = bw * bh;
            AScratch sc = aScratch.get();
            sc.b1 = aGrow(sc.b1, n * 3);
            sc.b2 = aGrow(sc.b2, n);
            sc.b3 = aGrow(sc.b3, n * 3);
            aCropInto(bgr, b, 3, sc.b1);
            aCropInto(blemishes, b, 1, sc.b2);
            sc.m1.create(bh, bw, CvType.CV_8UC3);
            sc.m1.put(0, 0, sc.b1);
            sc.m2.create(bh, bw, CvType.CV_8UC1);
            sc.m2.put(0, 0, sc.b2);
            inpaintTelea(sc.m1, sc.m2, sc.m3);
            sc.m3.get(0, 0, sc.b3);
            byte[] fd = sc.b3;
            for (int y = 0; y < bh; y++) for (int x = 0; x < bw; x++) {
                int i = (y + b.top) * W + x + b.left;
                if (blemishes[i] != 0 && groups[i] == b.label) for (int c = 0; c < 3; c++) repaired[i * 3 + c] = fd[(y * bw + x) * 3 + c];
            }
        });
        runAll(jobs);
        int size = 2 * (smoothSize / 2) + 1;
        aChanged = aGrowI(aChanged, total);
        if (size > 1) {
            aAroundIn = aMat(aAroundIn, H, W, CvType.CV_8UC1);
            aAroundIn.put(0, 0, blemishes);
            if (aAroundOut == null) aAroundOut = new Mat();
            long s = tic();
            Imgproc.dilate(aAroundIn, aAroundOut, aRing3());
            toc(s);
            aAround = aBuf(aAround, total);
            final byte[] around = aAround;
            aGetBytesInto(aAroundOut, around);
            for (int i = 0; i < total; i++) around[i] = (byte) (around[i] != 0 && filled[i] != 0 ? 1 : 0);
            double scale = 1.0 / (size * size);
            List<Box> aroundBoxes = paddedBoxes(around, size / 2);
            int[] aroundGroups = lastGroups;
            final int boxCount = aroundBoxes.size();
            final int[][] updateAt = new int[boxCount][];
            final byte[][] updateValue = new byte[boxCount][];
            final int window = size;
            List<Runnable> smoothJobs = new ArrayList<>();
            for (int bi = 0; bi < boxCount; bi++) {
                final int fb = bi;
                final Box b = aroundBoxes.get(bi);
                smoothJobs.add(() -> {
                    int bw = b.right - b.left, bh = b.bottom - b.top, bn = bw * bh;
                    AScratch sc = aScratch.get();
                    sc.b1 = aGrow(sc.b1, bn * 3);
                    sc.b2 = aGrow(sc.b2, bn);
                    sc.i1 = aGrowI(sc.i1, bn * 3);
                    sc.i2 = aGrowI(sc.i2, bn);
                    aCropInto(repaired, b, 3, sc.b1);
                    aCropInto(filled, b, 1, sc.b2);
                    for (int k = 0; k < bn; k++) sc.b2[k] = (byte) (sc.b2[k] != 0 ? 1 : 0);
                    Mat masked = new Mat();
                    try {
                        sc.m1.create(bh, bw, CvType.CV_8UC3);
                        sc.m1.put(0, 0, sc.b1);
                        sc.m2.create(bh, bw, CvType.CV_8UC1);
                        sc.m2.put(0, 0, sc.b2);
                        Core.bitwise_and(sc.m1, sc.m1, masked, sc.m2);
                        Imgproc.boxFilter(masked, sc.m3, CvType.CV_32S, new Size(window, window), new Point(-1, -1), false, Core.BORDER_REFLECT_101);
                        Imgproc.boxFilter(sc.m2, sc.m4, CvType.CV_32S, new Size(window, window), new Point(-1, -1), false, Core.BORDER_REFLECT_101);
                        sc.m3.get(0, 0, sc.i1);
                        sc.m4.get(0, 0, sc.i2);
                    } finally {
                        masked.release();
                    }
                    int[] cs = sc.i1, ws = sc.i2;
                    int n = 0;
                    int[] atTmp = new int[bn];
                    byte[] valTmp = new byte[bn * 3];
                    for (int y = 0; y < bh; y++) for (int x = 0; x < bw; x++) {
                        int i = (y + b.top) * W + x + b.left;
                        if (around[i] == 0 || aroundGroups[i] != b.label) continue;
                        float weight = (float) (ws[y * bw + x] * scale);
                        for (int c = 0; c < 3; c++) {
                            float total2 = (float) (cs[(y * bw + x) * 3 + c] * scale);
                            float v = total2 / Math.max(weight, 1e-6f) + 0.5f;
                            valTmp[n * 3 + c] = (byte) (int) Math.min(Math.max(v, 0f), 255f);
                        }
                        atTmp[n++] = i;
                    }
                    updateAt[fb] = Arrays.copyOf(atTmp, n);
                    updateValue[fb] = Arrays.copyOf(valTmp, n * 3);
                });
            }
            runAll(smoothJobs);
            int changed = 0;
            int[] ch = aChanged;
            for (int bi = 0; bi < boxCount; bi++) {
                int[] at = updateAt[bi];
                byte[] values = updateValue[bi];
                for (int q = 0; q < at.length; q++) {
                    int i = at[q];
                    repaired[i * 3] = values[q * 3]; repaired[i * 3 + 1] = values[q * 3 + 1]; repaired[i * 3 + 2] = values[q * 3 + 2];
                    if (repaired[i * 3] != bgr[i * 3] || repaired[i * 3 + 1] != bgr[i * 3 + 1] || repaired[i * 3 + 2] != bgr[i * 3 + 2]) ch[changed++] = i;
                }
            }
            aChangedCount = changed;
        } else {
            int changed = 0;
            int[] ch = aChanged;
            for (int i = 0; i < total; i++) {
                if (blemishes[i] == 0) continue;
                if (repaired[i * 3] != bgr[i * 3] || repaired[i * 3 + 1] != bgr[i * 3 + 1] || repaired[i * 3 + 2] != bgr[i * 3 + 2]) ch[changed++] = i;
            }
            aChangedCount = changed;
        }
        return repaired;
    }

    static Mat aRing3;
    static Mat aRing3() {
        if (aRing3 == null) aRing3 = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 3));
        return aRing3;
    }

    /** L channel of the smoothed image: lastL (white-balanced image) with only the changed pixels re-converted. */
    static byte[] aSmoothL(byte[] smoothed) {
        int total = H * W;
        aSmoothLBuf = aBuf(aSmoothLBuf, total);
        byte[] out = aSmoothLBuf;
        System.arraycopy(lastL, 0, out, 0, total);
        int n = aChangedCount;
        if (n == 0) return out;
        aPixBuf = aGrow(aPixBuf == null ? new byte[0] : aPixBuf, n * 3);
        aPixLabBuf = aGrow(aPixLabBuf == null ? new byte[0] : aPixLabBuf, n * 3);
        int[] ch = aChanged;
        byte[] px = aPixBuf;
        for (int k = 0; k < n; k++) {
            int i = ch[k] * 3;
            px[k * 3] = smoothed[i]; px[k * 3 + 1] = smoothed[i + 1]; px[k * 3 + 2] = smoothed[i + 2];
        }
        aPixIn = aMat(aPixIn, 1, n, CvType.CV_8UC3);
        aPixIn.put(0, 0, px);
        if (aPixOut == null) aPixOut = new Mat();
        long s = tic();
        Imgproc.cvtColor(aPixIn, aPixOut, Imgproc.COLOR_BGR2Lab);
        toc(s);
        byte[] lab = aPixLabBuf;
        aPixOut.get(0, 0, lab);
        for (int k = 0; k < n; k++) out[ch[k]] = lab[k * 3];
        return out;
    }

    static List<MatOfPoint> acceptedContours = new ArrayList<>();
    static List<double[]> accepted = new ArrayList<>();
    static List<Double> acceptedPct = new ArrayList<>(), rejectedPct = new ArrayList<>();
    static List<MatOfPoint> rejectedContours = new ArrayList<>();

    static void singleBalls(byte[] filled) {
        acceptedContours = new ArrayList<>();
        accepted = new ArrayList<>();
        acceptedPct = new ArrayList<>();
        rejectedContours = new ArrayList<>();
        rejectedPct = new ArrayList<>();
        List<MatOfPoint> contours = new ArrayList<>();
        long s = tic();
        findContoursTracked(bytesMat(filled, H, W), contours, tmp(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        toc(s);
        for (MatOfPoint contour : contours) {
            s = tic();
            double area = Imgproc.contourArea(contour);
            toc(s);
            int n = (int) contour.total();
            if (area < singleMinAreaPx || n < 5) continue;
            int[] pts = new int[n * 2];
            contour.get(0, 0, pts);
            double[] xs = new double[n], ys = new double[n];
            for (int i = 0; i < n; i++) { xs[i] = pts[i * 2]; ys[i] = pts[i * 2 + 1]; }
            double[] c = weightedCircle(xs, ys, null, null);
            if (c == null) continue;
            double tol = tolerance(c[2]);
            int on = 0;
            for (int i = 0; i < n; i++) if (Math.abs(dist(xs[i] - c[0], ys[i] - c[1]) - c[2]) <= tol) on++;
            double pct = (double) on / n;
            if (pct >= fitMinPctg) { acceptedContours.add(contour); accepted.add(c); acceptedPct.add(pct); }
            else { rejectedContours.add(contour); rejectedPct.add(pct); }
        }
    }

    static float[] eLevel = new float[0];
    static int[] eBucketCount = new int[0], ePlace = new int[0];

    static float[] brightnessRanks(byte[] brightness, byte[] remaining) {
        int[] cnt = new int[1];
        int[] labels = connected(remaining, 8, cnt, 1);
        int count = cnt[0];
        int bucketLen = count * 256 + 1;
        if (eBucketCount.length < bucketLen) { eBucketCount = new int[bucketLen]; ePlace = new int[bucketLen]; }
        int[] bucketCount = eBucketCount;
        Arrays.fill(bucketCount, 0, bucketLen, 0);
        int total = 0;
        for (int i = 0; i < H * W; i++) if (remaining[i] != 0) { bucketCount[labels[i] * 256 + (brightness[i] & 255) + 1]++; total++; }
        for (int k = 0; k < count * 256; k++) bucketCount[k + 1] += bucketCount[k];
        int[] groupStart = new int[count + 1];
        for (int g = 0; g <= count; g++) groupStart[g] = g < count ? bucketCount[g * 256] : total;
        int[] place = ePlace;
        System.arraycopy(bucketCount, 0, place, 0, bucketLen);
        if (eLevel.length != H * W) eLevel = new float[H * W];
        float[] level = eLevel;
        Arrays.fill(level, -1f);
        for (int i = 0; i < H * W; i++) {
            if (remaining[i] == 0) continue;
            int g = labels[i];
            int rank = place[g * 256 + (brightness[i] & 255)]++ - groupStart[g];
            int size = groupStart[g + 1] - groupStart[g];
            level[i] = (float) ((double) rank / Math.max(size - 1, 1));
        }
        return level;
    }

    static short[][] runBuf = new short[8][0];
    static boolean[] eFound = new boolean[0];

    static boolean[] valleyLines(float[] level, Expect expected) {
        int[] reachByRow = new int[H];
        for (int y = 0; y < H; y++) reachByRow[y] = expected == null ? Math.max(valleyFallbackReachPx, 1) : Math.max((int) Math.rint(valleyReachPctg * expected.at(y)), 1);
        final float depth = (float) (valleyDepthPctg / 100), noise = (float) (valleyNoisePctg / 100);
        final int[] stepX = {1, 1, 0, -1, -1, -1, 0, 1}, stepY = {0, 1, 1, 1, 0, -1, -1, -1};
        if (eFound.length != H * W) eFound = new boolean[H * W];
        boolean[] found = eFound;
        Arrays.fill(found, false);
        final int n = insideCount;
        final int[] idx = insideIdx;
        List<Runnable> jobs = new ArrayList<>();
        for (int d = 0; d < 8; d++) {
            if (runBuf[d].length < H * W) runBuf[d] = new short[H * W];
            final short[] run = runBuf[d];
            final int dx = stepX[d], dy = stepY[d], off = dy * W + dx;
            jobs.add(() -> {
                for (int q = 0; q < n; q++) {
                    int i = idx[off > 0 ? n - 1 - q : q];
                    int y = i / W, x = i - y * W, ny = y + dy, nx = x + dx;
                    boolean ok = false;
                    if (ny >= 0 && ny < H && nx >= 0 && nx < W) {
                        float following = level[i + off];
                        ok = following >= 0 && following >= level[i] - noise;
                    }
                    run[i] = ok ? (short) Math.min(run[i + off] + 1, Short.MAX_VALUE) : 0;
                }
            });
        }
        runAll(jobs);
        boolean[] rising = new boolean[8];
        int[] offsets = new int[8];
        for (int d = 0; d < 8; d++) offsets[d] = stepY[d] * W + stepX[d];
        for (int k = 0; k < n; k++) {
            int i = idx[k];
            int reach = reachByRow[i / W];
            float start = level[i];
            if (start < 0) continue;
            for (int d = 0; d < 8; d++) rising[d] = runBuf[d][i] >= reach && level[i + reach * offsets[d]] - start > depth;
            boolean hit = false;
            for (int first = 0; first < 8 && !hit; first++) for (int gap = 3; gap <= 5; gap++) { int second = (first + gap) % 8; if (second > first && rising[first] && rising[second]) { hit = true; break; } }
            found[i] = hit;
        }
        return found;
    }

    static float[] gauss(float[] data, int rows, int cols, double sigma) {
        return gaussOwn(data, rows, cols, sigma);
    }

    /** B: per-thread reusable Mats and float buffers (Mat.create only reallocates when the size changes). */
    static final class BScratch { final Mat a = new Mat(), b = new Mat(), c = new Mat(); float[] in = new float[0], out = new float[0]; }
    static final ThreadLocal<BScratch> bScratch = new ThreadLocal<BScratch>() { @Override protected BScratch initialValue() { return new BScratch(); } };

    /** Blurs data (rows x cols) into outBuf (capacity >= rows*cols) without allocating. */
    static void bGaussInto(float[] data, int rows, int cols, double sigma, float[] outBuf) {
        BScratch s = bScratch.get();
        s.a.create(rows, cols, CvType.CV_32FC1);
        s.a.put(0, 0, data);
        Imgproc.GaussianBlur(s.a, s.b, new Size(0, 0), sigma);
        s.b.get(0, 0, outBuf);
    }

    static float[] gaussOwn(float[] data, int rows, int cols, double sigma) {
        float[] result = new float[rows * cols];
        bGaussInto(data, rows, cols, sigma, result);
        return result;
    }

    static float[] sobelOwn(float[] data, int rows, int cols, int dx, int dy, int ksize) {
        BScratch s = bScratch.get();
        s.a.create(rows, cols, CvType.CV_32FC1);
        s.a.put(0, 0, data);
        Imgproc.Sobel(s.a, s.b, CvType.CV_32F, dx, dy, ksize);
        float[] result = new float[rows * cols];
        s.b.get(0, 0, result);
        return result;
    }

    /** Sobel followed by Gaussian, kept native in between (identical to gaussOwn(sobelOwn(..))). */
    static float[] bSobelGaussOwn(float[] data, int rows, int cols, int dx, int dy, int ksize, double sigma) {
        BScratch s = bScratch.get();
        s.a.create(rows, cols, CvType.CV_32FC1);
        s.a.put(0, 0, data);
        Imgproc.Sobel(s.a, s.b, CvType.CV_32F, dx, dy, ksize);
        Imgproc.GaussianBlur(s.b, s.c, new Size(0, 0), sigma);
        float[] result = new float[rows * cols];
        s.c.get(0, 0, result);
        return result;
    }

    static java.util.concurrent.ExecutorService pool;
    static int poolSize;

    static void runAll(List<Runnable> jobs) {
        int threads = Math.max(1, THREADS);
        if (threads == 1 || jobs.size() <= 1) { for (Runnable job : jobs) job.run(); return; }
        if (pool == null || poolSize != threads) {
            if (pool != null) pool.shutdown();
            pool = java.util.concurrent.Executors.newFixedThreadPool(threads, task -> { Thread t = new Thread(task, "Detect worker"); t.setDaemon(true); return t; });
            poolSize = threads;
        }
        List<java.util.concurrent.Future<?>> futures = new ArrayList<>();
        for (Runnable job : jobs) futures.add(pool.submit(job));
        try {
            for (java.util.concurrent.Future<?> f : futures) f.get();
        } catch (InterruptedException error) {
            for (java.util.concurrent.Future<?> f : futures) f.cancel(true);
            Thread.currentThread().interrupt();
            throw new RuntimeException(error);
        } catch (java.util.concurrent.ExecutionException error) {
            for (java.util.concurrent.Future<?> f : futures) f.cancel(true);
            Throwable cause = error.getCause();
            if (cause instanceof RuntimeException) throw (RuntimeException) cause;
            if (cause instanceof Error) throw (Error) cause;
            throw new RuntimeException(cause);
        }
    }

    static float[] sobel(float[] data, int rows, int cols, int dx, int dy, int ksize) {
        Mat m = floatsMat(data, rows, cols), out = tmp();
        long s = tic();
        Imgproc.Sobel(m, out, CvType.CV_32F, dx, dy, ksize);
        toc(s);
        return getFloats(out);
    }

    static double[][] outlineNormals(boolean[] inside, byte[] insideBytes) {
        Box b = insideBox(10);
        if (b == null) return new double[][]{new double[0], new double[0], new double[0], new double[0]};
        int bw = b.right - b.left, bh = b.bottom - b.top;
        float[] f = new float[bw * bh];
        byte[] ib = new byte[bw * bh];
        for (int y = 0; y < bh; y++) for (int x = 0; x < bw; x++) if (inside[(y + b.top) * W + x + b.left]) { f[y * bw + x] = 1f; ib[y * bw + x] = 1; }
        final float[] soft = gauss(f, bh, bw, 1.5);
        final float[][] grad = new float[2][];
        final byte[] er = new byte[bw * bh];
        runAll(Arrays.<Runnable>asList(
                () -> grad[0] = sobelOwn(soft, bh, bw, 1, 0, 3),
                () -> grad[1] = sobelOwn(soft, bh, bw, 0, 1, 3),
                () -> {
                    Mat em = new Mat(bh, bw, CvType.CV_8UC1), eroded = new Mat(), kernel = Mat.ones(3, 3, CvType.CV_8U);
                    try {
                        em.put(0, 0, ib);
                        Imgproc.erode(em, eroded, kernel);
                        eroded.get(0, 0, er);
                    } finally {
                        em.release();
                        eroded.release();
                        kernel.release();
                    }
                }));
        float[] gx = grad[0], gy = grad[1];
        int n = 0;
        for (int i = 0, total = bw * bh; i < total; i++) if (ib[i] != 0 && er[i] == 0) n++;
        double[][] out = new double[4][n];
        int k = 0;
        for (int y = 0; y < bh; y++) for (int x = 0; x < bw; x++) {
            int i = y * bw + x;
            if (ib[i] != 0 && er[i] == 0) {
                double len = Math.max(dist(gx[i], gy[i]), 1e-6);
                out[0][k] = x + b.left; out[1][k] = y + b.top; out[2][k] = (float) (gx[i] / len); out[3][k] = (float) (gy[i] / len);
                k++;
            }
        }
        return out;
    }

    static double[][] valleyNormals(boolean[] inside, byte[] insideBytes, float[] level, boolean[] valleys) {
        Box b = insideBox(24);
        if (b == null) return new double[][]{new double[0], new double[0], new double[0], new double[0]};
        int bw = b.right - b.left, bh = b.bottom - b.top;
        float[] f = new float[bw * bh], v = new float[bw * bh];
        for (int y = 0; y < bh; y++) for (int x = 0; x < bw; x++) { int i = (y + b.top) * W + x + b.left; if (inside[i]) { f[y * bw + x] = 1f; v[y * bw + x] = level[i]; } }
        final float[][] pair = new float[2][];
        runAll(Arrays.<Runnable>asList(() -> pair[0] = gaussOwn(f, bh, bw, 1.5), () -> pair[1] = gaussOwn(v, bh, bw, 1.5)));
        float[] weight = pair[0], blurred = pair[1];
        final float[] height = new float[bw * bh];
        for (int i = 0; i < bw * bh; i++) height[i] = f[i] != 0 ? blurred[i] / Math.max(weight[i], 1e-6f) : 0f;
        final float[][] slopes = new float[2][];
        runAll(Arrays.<Runnable>asList(() -> slopes[0] = bSobelGaussOwn(height, bh, bw, 1, 0, 5, 2), () -> slopes[1] = bSobelGaussOwn(height, bh, bw, 0, 1, 5, 2)));
        float[] slopeX = slopes[0], slopeY = slopes[1];
        int n = 0;
        for (int y = 0; y < bh; y++) { int g = (y + b.top) * W + b.left; for (int x = 0; x < bw; x++) if (valleys[g + x] && inside[g + x]) n++; }
        double[][] out = new double[4][n];
        int k = 0;
        for (int y = 0; y < bh; y++) for (int x = 0; x < bw; x++) {
            int i = y * bw + x, g = (y + b.top) * W + x + b.left;
            if (valleys[g] && inside[g]) {
                double len = Math.max(dist(slopeX[i], slopeY[i]), 1e-6);
                out[0][k] = x + b.left; out[1][k] = y + b.top; out[2][k] = (float) (slopeX[i] / len); out[3][k] = (float) (slopeY[i] / len);
                k++;
            }
        }
        return out;
    }

    static double[][] columns(List<double[]> rows) {
        int n = rows.size(), k = rows.isEmpty() ? 4 : rows.get(0).length;
        double[][] out = new double[k][n];
        for (int i = 0; i < n; i++) for (int j = 0; j < k; j++) out[j][i] = rows.get(i)[j];
        return out;
    }

    static float maxDistance(byte[] insideBytes, int maskSize) {
        Mat m = bytesMat(insideBytes, H, W), d = tmp();
        long s = tic();
        Imgproc.distanceTransform(m, d, Imgproc.DIST_L2, maskSize);
        double max = Core.minMaxLoc(d).maxVal;
        toc(s);
        return (float) max;
    }

    static List<double[]> spacedPeaks(float[] coverage, double minimum, float[] radiusMap, double[] rowRadius, byte[] insideImage, double spacing, double minFill) {
        // B: peaks are pixel indices packed with a sortable (descending, stable) coverage key; no boxed lists.
        long[] keys = new long[64];
        int np = 0;
        if (minimum > 0) {
            for (int k = 0; k < insideCount; k++) {
                int i = insideIdx[k];
                float v = coverage[i];
                if (!(v >= minimum)) continue;
                int y = i / W, x = i - y * W;
                float most = v;
                for (int yy = Math.max(y - 1, 0); yy < Math.min(y + 2, H); yy++) for (int xx = Math.max(x - 1, 0); xx < Math.min(x + 2, W); xx++) most = Math.max(most, coverage[yy * W + xx]);
                if (v >= most) { if (np == keys.length) keys = Arrays.copyOf(keys, np * 2); keys[np++] = bPeakKey(v, i); }
            }
        } else {
            Mat cm = floatsMat(coverage, H, W), dil = tmp();
            long s = tic();
            Mat ones3 = Mat.ones(3, 3, CvType.CV_8U);
            Imgproc.dilate(cm, dil, ones3);
            ones3.release();
            toc(s);
            float[] d = getFloats(dil);
            for (int i = 0; i < H * W; i++) if (coverage[i] >= d[i] && coverage[i] >= minimum) { if (np == keys.length) keys = Arrays.copyOf(keys, np * 2); keys[np++] = bPeakKey(coverage[i], i); }
        }
        Arrays.sort(keys, 0, np);
        double[] kx = new double[16], ky = new double[16], kr = new double[16];
        int kn = 0;
        for (int q = 0; q < np; q++) {
            int i = (int) keys[q];
            int y = i / W, x = i - y * W;
            double radius = radiusMap != null ? radiusMap[i] : rowRadius[y];
            boolean clear = true;
            for (int o = 0; o < kn; o++) if (dist(x - kx[o], y - ky[o]) < spacing * Math.max(radius, kr[o])) { clear = false; break; }
            if (!clear) continue;
            if (minFill > 0 && circleFill(insideImage, x, y, radius) < minFill) continue;
            if (kn == kx.length) { kx = Arrays.copyOf(kx, kn * 2); ky = Arrays.copyOf(ky, kn * 2); kr = Arrays.copyOf(kr, kn * 2); }
            kx[kn] = x; ky[kn] = y; kr[kn] = radius; kn++;
        }
        List<double[]> kept = new ArrayList<>(kn);
        for (int o = 0; o < kn; o++) kept.add(new double[]{kx[o], ky[o], kr[o]});
        return kept;
    }

    /** Ascending order of this key = coverage descending (Float.compare order), ties by ascending pixel index. */
    static long bPeakKey(float v, int i) {
        int bits = Float.floatToIntBits(v);
        int sortable = bits ^ ((bits >> 31) & 0x7fffffff);
        return ((long) ~sortable << 32) | i;
    }

    static byte[] bInsideImage = new byte[0];
    /** Shared 0/255 image of the inside pixels (rebuilt per call from insideIdx; never stored in debug fields). */
    static byte[] bInsideImage() {
        if (bInsideImage.length != H * W) bInsideImage = new byte[H * W]; else Arrays.fill(bInsideImage, (byte) 0);
        for (int k = 0; k < insideCount; k++) bInsideImage[insideIdx[k]] = (byte) 255;
        return bInsideImage;
    }
    static float[] bBest = new float[0], bBestRadius = new float[0], bCoverage = new float[0], bClimb = new float[0];
    static float[] bZeroed(float[] a) { if (a.length != H * W) return new float[H * W]; Arrays.fill(a, 0f); return a; }

    static List<double[][]> groupSources(Box b, int[] groups, boolean[] inside, List<double[][]> sources) {
        List<double[][]> chosen = new ArrayList<>();
        for (double[][] src : sources) {
            int len = src[0].length, n = 0;
            for (int i = 0; i < len; i++) {
                int x = (int) src[0][i], y = (int) src[1][i];
                if (y >= b.top && y < b.bottom && x >= b.left && x < b.right && groups[y * W + x] == b.label && inside[y * W + x]) n++;
            }
            double[][] cols = new double[5][n];
            int k = 0;
            for (int i = 0; i < len; i++) {
                int x = (int) src[0][i], y = (int) src[1][i];
                if (y >= b.top && y < b.bottom && x >= b.left && x < b.right && groups[y * W + x] == b.label && inside[y * W + x]) {
                    cols[0][k] = src[0][i]; cols[1][k] = src[1][i]; cols[2][k] = src[2][i]; cols[3][k] = src[3][i];
                    cols[4][k] = src.length > 4 ? src[4][i] : 0;
                    k++;
                }
            }
            chosen.add(cols);
        }
        return chosen;
    }

    static float[] bVotesFlat = new float[0];
    static Expect ballSizeByRow(boolean[] inside, byte[] insideBytes, float[] level, boolean[] valleys, double[][] outline) {
        List<Double> rows = new ArrayList<>(), radii = new ArrayList<>();
        boolean any = insideCount > 0;
        byte[] insideImage = any ? bInsideImage() : null;
        if (any) {
            List<double[][]> sources = new ArrayList<>();
            List<Float> weights = new ArrayList<>();
            sources.add(outline);
            weights.add(1f);
            double[][] vn = valleyNormals(inside, insideBytes, level, valleys);
            stage("seeds: valley normals");
            double[][] neg = new double[][]{vn[0], vn[1], new double[vn[2].length], new double[vn[3].length]};
            for (int i = 0; i < vn[2].length; i++) { neg[2][i] = -vn[2][i]; neg[3][i] = -vn[3][i]; }
            sources.add(vn);
            weights.add((float) sizeVoteValleyWeight);
            sources.add(neg);
            weights.add((float) sizeVoteValleyWeight);
            double largest = Math.max(maxDistance(insideBytes, 5) * 1.3, sizeVoteSmallestPx + 1);
            int margin = (int) Math.ceil(largest) + (int) Math.rint(4 * Math.max(0.15 * largest, 1.0)) + 3;
            List<Double> trial = new ArrayList<>();
            for (double r = sizeVoteSmallestPx; r <= largest; r *= 1 + sizeVoteStepPctg) trial.add(r);
            float[] best = bBest = bZeroed(bBest), bestRadius = bBestRadius = bZeroed(bBestRadius);
            List<Box> boxes = paddedBoxes(insideBytes, margin);
            int[] groups = lastGroups;
            int boxCount = boxes.size(), trials = trial.size();
            final List<List<double[][]>> boxSources = new ArrayList<>();
            final int[][] boxX = new int[boxCount][], boxY = new int[boxCount][], boxEdges = new int[boxCount][];
            for (int bi = 0; bi < boxCount; bi++) {
                Box b = boxes.get(bi);
                boxSources.add(groupSources(b, groups, inside, sources));
                int count = 0, ot = H, ob = -1, ol = W, or = -1;
                for (int y = b.top; y < b.bottom; y++) for (int x = b.left; x < b.right; x++) { int i = y * W + x; if (groups[i] == b.label && inside[i]) { count++; ot = Math.min(ot, y); ob = Math.max(ob, y); ol = Math.min(ol, x); or = Math.max(or, x); } }
                int[] ox = new int[count], oy = new int[count];
                int k = 0;
                for (int y = b.top; y < b.bottom; y++) for (int x = b.left; x < b.right; x++) { int i = y * W + x; if (groups[i] == b.label && inside[i]) { ox[k] = x; oy[k] = y; k++; } }
                boxX[bi] = ox; boxY[bi] = oy; boxEdges[bi] = new int[]{ot, ob, ol, or};
            }
            final int[] valOff = new int[boxCount + 1];
            for (int bi = 0; bi < boxCount; bi++) valOff[bi + 1] = valOff[bi] + boxX[bi].length * trials;
            if (bVotesFlat.length < valOff[boxCount]) bVotesFlat = new float[valOff[boxCount]];
            final float[] votesFlat = bVotesFlat;
            final float[] weightList = new float[weights.size()];
            for (int si = 0; si < weightList.length; si++) weightList[si] = weights.get(si);
            List<Runnable> jobs = new ArrayList<>();
            for (int bi = 0; bi < boxCount; bi++) for (int ti = 0; ti < trials; ti++) {
                final int fb = bi, ft = ti;
                final Box b = boxes.get(bi);
                final double radius = trial.get(ti);
                jobs.add(() -> {
                    List<double[][]> local = boxSources.get(fb);
                    int[] ox = boxX[fb], oy = boxY[fb], e = boxEdges[fb];
                    int count = ox.length;
                    double sigma = Math.max(0.15 * radius, 1.0);
                    int reach = (((int) Math.rint(sigma * 8 + 1)) | 1) / 2 + 2;
                    int top = Math.max(e[0] - reach, b.top), bottom = Math.min(e[1] + reach + 1, b.bottom), left = Math.max(e[2] - reach, b.left), right = Math.min(e[3] + reach + 1, b.right);
                    int rw = right - left, rh = bottom - top;
                    BScratch sc = bScratch.get();
                    int need = rw * rh;
                    if (sc.in.length < need) sc.in = new float[need]; else Arrays.fill(sc.in, 0, need, 0f);
                    float[] votes = sc.in;
                    boolean hasVotes = false;
                    float rf = (float) radius;
                    for (int si = 0; si < local.size(); si++) {
                        double[][] src = local.get(si);
                        double[] sx = src[0], sy = src[1], sdx = src[2], sdy = src[3];
                        float wgt = weightList[si];
                        int n = sx.length;
                        for (int i = 0; i < n; i++) {
                            int cx = (int) Math.rint(sx[i] + (double) ((float) sdx[i] * rf)), cy = (int) Math.rint(sy[i] + (double) ((float) sdy[i] * rf));
                            if (cx >= left && cx < right && cy >= top && cy < bottom) { votes[(cy - top) * rw + cx - left] += wgt; hasVotes = true; }
                        }
                    }
                    int vo = valOff[fb] + ft * count;
                    if (hasVotes) {
                        if (sc.out.length < need) sc.out = new float[need];
                        float[] blurred = sc.out;
                        bGaussInto(votes, rh, rw, sigma, blurred);
                        float c1 = (float) (2 * Math.PI * sigma * sigma), c2 = (float) (2 * Math.PI * radius);
                        for (int j = 0; j < count; j++) votesFlat[vo + j] = blurred[(oy[j] - top) * rw + ox[j] - left] * c1 / c2;
                    } else Arrays.fill(votesFlat, vo, vo + count, 0f);
                });
            }
            runAll(jobs);
            for (int bi = 0; bi < boxCount; bi++) {
                int[] ox = boxX[bi], oy = boxY[bi];
                int count = ox.length;
                for (int ti = 0; ti < trials; ti++) {
                    int vo = valOff[bi] + ti * count;
                    float radius = (float) (double) trial.get(ti);
                    for (int j = 0; j < count; j++) { float v = votesFlat[vo + j]; int pi = oy[j] * W + ox[j]; if (v > best[pi]) { best[pi] = v; bestRadius[pi] = radius; } }
                }
            }
            stage("seeds: radius loop");
            for (double[] p : spacedPeaks(best, sizeVoteMinCoverPctg, bestRadius, null, insideImage, 0.8, 0.8)) if (p[2] >= sizeSampleMinPx) { rows.add(p[1]); radii.add(p[2]); }
        }
        for (double[] c : accepted) if (c[2] >= sizeSampleMinPx) { rows.add(c[1]); radii.add(c[2]); }
        if (rows.size() < 2) {
            List<double[]> refs = new ArrayList<>();
            for (double[] c : accepted) refs.add(new double[]{c[1], c[2]});
            return radiusByRow(refs);
        }
        return trimmedRadiusLine(rows, radii);
    }

    static int climbToTop(float[] values, int x, int y) {
        while (true) {
            double best = values[y * W + x];
            int nx = x, ny = y;
            for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++) {
                if (dy == 0 && dx == 0) continue;
                int yy = y + dy, xx = x + dx;
                if (yy >= 0 && yy < H && xx >= 0 && xx < W && values[yy * W + xx] > best) { best = values[yy * W + xx]; nx = xx; ny = yy; }
            }
            if (nx == x && ny == y) return y * W + x;
            x = nx; y = ny;
        }
    }

    static Expect seedExpected;
    static float[] lastCoverage = new float[0];
    static int[] spotX, spotY;

    static void seedPoints(boolean[] inside, byte[] insideBytes, float[] level, boolean[] valleys) {
        boolean any = false;
        for (boolean b : inside) if (b) { any = true; break; }
        if (!any) { spotX = new int[0]; spotY = new int[0]; seedExpected = null; return; }
        double[][] outline = outlineNormals(inside, insideBytes);
        stage("seeds: outline normals");
        Expect expected = ballSizeByRow(inside, insideBytes, level, valleys, outline);
        stage("seeds: ball size votes");
        seedExpected = expected;
        double[] rowRadius = new double[H];
        if (expected == null) { double fixed = Math.max(maxDistance(insideBytes, 5), 2.0); Arrays.fill(rowRadius, fixed); }
        else for (int y = 0; y < H; y++) rowRadius[y] = expected.at(y);
        int n = outline[0].length;
        double sigma = 1.0, maxR = 0;
        if (n > 0) {
            double[] scaled = new double[n];
            for (int i = 0; i < n; i++) { scaled[i] = seedVoteBlurPctg * rowRadius[(int) outline[1][i]]; maxR = Math.max(maxR, rowRadius[(int) outline[1][i]]); }
            sigma = Math.max(median(scaled, n), 1.0);
        }
        int margin = (int) Math.ceil(n > 0 ? 1.1 * maxR : 0) + (int) Math.rint(4 * sigma) + 3;
        float[] coverage = bCoverage = bZeroed(bCoverage);
        List<Box> boxes = paddedBoxes(insideBytes, margin);
        int[] groups = lastGroups;
        float third = (float) (1.0 / 3);
        float c1 = (float) (2 * Math.PI * sigma * sigma);
        final double voteSigma = sigma;
        List<Runnable> jobs = new ArrayList<>();
        for (final Box b : boxes) jobs.add(() -> {
            List<double[][]> local = groupSources(b, groups, inside, Collections.singletonList(outline));
            double[][] src = local.get(0);
            // B: only pixels of this group's inside area are read back, so blur just their bounding box grown by the kernel reach
            // (a window of any read pixel stays inside it; where it is cut by the box edge the border handling is the same as before)
            int et = H, eb = -1, el = W, er = -1;
            for (int y = b.top; y < b.bottom; y++) for (int x = b.left; x < b.right; x++) { int i = y * W + x; if (groups[i] == b.label && inside[i]) { if (y < et) et = y; eb = y; if (x < el) el = x; if (x > er) er = x; } }
            if (eb < 0) return;
            int reach = (((int) Math.rint(voteSigma * 8 + 1)) | 1) / 2 + 2;
            int top = Math.max(et - reach, b.top), bottom = Math.min(eb + reach + 1, b.bottom), left = Math.max(el - reach, b.left), right = Math.min(er + reach + 1, b.right);
            int bw = right - left, bh = bottom - top;
            BScratch sc = bScratch.get();
            int need = bw * bh;
            if (sc.in.length < need) sc.in = new float[need]; else Arrays.fill(sc.in, 0, need, 0f);
            float[] votes = sc.in;
            boolean hasVotes = false;
            double[] sx = src[0], sy = src[1], sdx = src[2], sdy = src[3];
            int sn = sx.length;
            for (double factor : new double[]{0.9, 1.0, 1.1}) {
                for (int i = 0; i < sn; i++) {
                    double dist = rowRadius[(int) sy[i]] * factor;
                    int cx = (int) Math.rint(sx[i] + sdx[i] * dist), cy = (int) Math.rint(sy[i] + sdy[i] * dist);
                    if (cx >= left && cx < right && cy >= top && cy < bottom) { votes[(cy - top) * bw + cx - left] += third; hasVotes = true; }
                }
            }
            float[] blurred = sc.out;
            if (hasVotes) {
                if (sc.out.length < need) sc.out = new float[need];
                blurred = sc.out;
                bGaussInto(votes, bh, bw, voteSigma, blurred);
            }
            for (int y = 0; y < bh; y++) for (int x = 0; x < bw; x++) {
                int i = (y + top) * W + x + left;
                if (groups[i] == b.label && inside[i]) coverage[i] = (float) (((hasVotes ? blurred[y * bw + x] : 0f) * c1) / (2 * Math.PI * rowRadius[y + top]));
            }
        });
        runAll(jobs);
        byte[] insideImage = bInsideImage();
        stage("seeds: coverage votes");
        lastCoverage = coverage;
        List<double[]> kept = spacedPeaks(coverage, seedMinCoverPctg, null, rowRadius, insideImage, seedSpacingPctg, seedMinFillPctg);
        stage("seeds: spaced peaks");
        float[] climb = climbValues(inside, insideBytes, level, seedClimbBlurPx);
        stage("seeds: climb");
        Map<Integer, int[]> firstOnTop = new HashMap<>();
        int[] seedIdx = new int[kept.size()];
        int c = 0;
        for (double[] k : kept) {
            int x = (int) k[0], y = (int) k[1];
            int top = climbToTop(climb, x, y);
            int[] other = firstOnTop.get(top);
            if (other != null) { if (dist(x - other[0], y - other[1]) < seedSameSpotReachPctg * k[2]) continue; }
            else firstOnTop.put(top, new int[]{x, y});
            seedIdx[c++] = y * W + x;
        }
        Arrays.sort(seedIdx, 0, c);
        spotX = new int[c]; spotY = new int[c];
        for (int k = 0; k < c; k++) { spotX[k] = seedIdx[k] % W; spotY[k] = seedIdx[k] / W; }
    }

    static float[] climbValues(boolean[] inside, byte[] insideBytes, float[] level, double blur) {
        if (bClimb.length != H * W) bClimb = new float[H * W];
        float[] result = bClimb;
        Arrays.fill(result, Float.NEGATIVE_INFINITY);
        Box b = insideBox((int) Math.rint(4 * Math.max(blur, 0)) + 4);
        if (b == null) return result;
        int bw = b.right - b.left, bh = b.bottom - b.top;
        float[] f = new float[bw * bh], v = new float[bw * bh];
        for (int y = 0; y < bh; y++) for (int x = 0; x < bw; x++) { int i = (y + b.top) * W + x + b.left; if (inside[i]) { f[y * bw + x] = 1f; v[y * bw + x] = level[i]; } }
        float[] values = v;
        if (blur > 0) {
            final float[][] pair = new float[2][];
            runAll(Arrays.<Runnable>asList(() -> pair[0] = gaussOwn(f, bh, bw, blur), () -> pair[1] = gaussOwn(v, bh, bw, blur)));
            float[] weight = pair[0], blurred = pair[1];
            values = new float[bw * bh];
            for (int i = 0; i < bw * bh; i++) values[i] = blurred[i] / Math.max(weight[i], 1e-6f);
        }
        for (int y = 0; y < bh; y++) for (int x = 0; x < bw; x++) { int i = (y + b.top) * W + x + b.left; if (inside[i]) result[i] = values[y * bw + x]; }
        return result;
    }

    static double[] arcSpan(double[] xs, double[] ys, boolean[] keep, double cx, double cy) {
        int n = 0;
        for (int i = 0; i < xs.length; i++) if (keep == null || keep[i]) n++;
        double[] angles = new double[n];
        int k = 0;
        for (int i = 0; i < xs.length; i++) if (keep == null || keep[i]) angles[k++] = Math.toDegrees(Math.atan2(ys[i] - cy, xs[i] - cx));
        Arrays.sort(angles);
        int widest = 0;
        double widestGap = -1;
        for (int i = 0; i < n; i++) { double gap = (i + 1 < n ? angles[i + 1] : angles[0] + 360) - angles[i]; if (gap > widestGap) { widestGap = gap; widest = i; } }
        double start = angles[(widest + 1) % n], end = angles[widest];
        if (end < start) end += 360;
        return new double[]{start, end};
    }

    static Object[] fitArc(double[] xs, double[] ys, double px, double py, double fx, double fy, double bx, double by, double radius) {
        double len = Math.max(dist(fx - px, fy - py), 1e-6);
        double cx = px + (fx - px) / len * radius, cy = py + (fy - py) / len * radius;
        boolean[] keep = new boolean[xs.length];
        Arrays.fill(keep, true);
        int kept = xs.length;
        for (int it = 0; it < fitRefineRounds + 1; it++) {
            if (kept < arcMinPoints) return null;
            double[] c = fixedRadiusCenter(xs, ys, keep, cx, cy, radius);
            cx = c[0]; cy = c[1];
            kept = markOnCircle(xs, ys, cx, cy, radius, keep);
        }
        if ((fx - px) * (cx - px) + (fy - py) * (cy - py) <= 0 || (bx - px) * (cx - px) + (by - py) * (cy - py) >= 0 || kept < 2) return null;
        return new Object[]{cx, cy, keep};
    }

    static List<Conn> spotConnections(int[] clumps, Expect expected, boolean[] valleys, boolean fitArcs) {
        int[] spotClump = new int[spotX.length];
        for (int i = 0; i < spotX.length; i++) spotClump[i] = clumps[spotY[i] * W + spotX[i]];
        TreeSet<Integer> clumpIds = new TreeSet<>();
        for (int c : spotClump) clumpIds.add(c);
        // valley pixels grouped by clump (counting sort; ascending insideIdx order kept within each clump)
        int most = 0;
        for (int k = 0; k < insideCount; k++) { int i = insideIdx[k]; if (valleys[i]) most = Math.max(most, clumps[i]); }
        int[] vStart = new int[most + 2];
        for (int k = 0; k < insideCount; k++) { int i = insideIdx[k]; if (valleys[i]) vStart[clumps[i] + 1]++; }
        for (int c = 0; c <= most; c++) vStart[c + 1] += vStart[c];
        int[] vPix = new int[vStart[most + 1]];
        int[] vFill = vStart.clone();
        for (int k = 0; k < insideCount; k++) { int i = insideIdx[k]; if (valleys[i]) vPix[vFill[clumps[i]]++] = i; }
        List<Conn> conns = new ArrayList<>();
        for (int clump : clumpIds) {
            List<Integer> members = new ArrayList<>();
            for (int i = 0; i < spotX.length; i++) if (spotClump[i] == clump) members.add(i);
            int count = members.size();
            double[] sx = new double[count], sy = new double[count];
            for (int i = 0; i < count; i++) { sx[i] = spotX[members.get(i)]; sy[i] = spotY[members.get(i)]; }
            boolean known = clump >= 0 && clump <= most;
            int pFrom = known ? vStart[clump] : 0, np = known ? vStart[clump + 1] - pFrom : 0;
            double[] pxs = new double[np], pys = new double[np];
            for (int p = 0; p < np; p++) { int i = vPix[pFrom + p]; pxs[p] = i % W; pys[p] = i / W; }
            double[] along = new double[np];
            int[] nearIdx = new int[np];
            for (int first = 0; first < count; first++) for (int second = first + 1; second < count; second++) {
                double dx = sx[second] - sx[first], dy = sy[second] - sy[first];
                double length = dist(dx, dy);
                double mx = sx[first] + dx / 2, my = sy[first] + dy / 2;
                boolean blocked = false;
                for (int o = 0; o < count; o++) if (o != first && o != second && dist(sx[o] - mx, sy[o] - my) < length / 2) { blocked = true; break; }
                if (blocked || length < 1) continue;
                int samples = (int) length + 2, hits = 0;
                for (int sIdx = 0; sIdx < samples; sIdx++) {
                    double f = (double) sIdx / (samples - 1);
                    int qx = (int) Math.rint(sx[first] + f * dx), qy = (int) Math.rint(sy[first] + f * dy);
                    if (clumps[qy * W + qx] == clump) hits++;
                }
                if ((double) hits / samples < connectInsidePctg) continue;
                double radius = expected != null ? (expected.at(sy[first]) + expected.at(sy[second])) / 2 : length / 2;
                Conn c = new Conn();
                c.sx = sx[first]; c.sy = sy[first]; c.ex = sx[second]; c.ey = sy[second];
                c.pointX = mx; c.pointY = my;
                int nc = 0;
                for (int p = 0; p < pxs.length; p++) {
                    double ox = pxs[p] - sx[first], oy = pys[p] - sy[first];
                    double pos = (ox * dx + oy * dy) / (length * length);
                    double across = Math.abs(ox * dy - oy * dx) / length;
                    if (pos > guessEndSkipPctg && pos < 1 - guessEndSkipPctg && across <= radius) { along[nc] = pos; nearIdx[nc] = p; nc++; }
                }
                c.nx = new double[nc]; c.ny = new double[nc];
                for (int q = 0; q < nc; q++) { c.nx[q] = pxs[nearIdx[q]]; c.ny[q] = pys[nearIdx[q]]; }
                if (nc > 0) { double share = median(along, nc); c.pointX = sx[first] + share * dx; c.pointY = sy[first] + share * dy; c.guessed = true; }
                if (fitArcs && nc >= arcMinPoints) {
                    boolean lowerIsEnd = c.ey >= c.sy;
                    for (boolean frontIsEnd : new boolean[]{lowerIsEnd, !lowerIsEnd}) {
                        double fx = frontIsEnd ? c.ex : c.sx, fy = frontIsEnd ? c.ey : c.sy, bx = frontIsEnd ? c.sx : c.ex, by = frontIsEnd ? c.sy : c.ey;
                        double arcRadius = expected != null ? expected.at(fy) : length / 2;
                        Object[] res = fitArc(c.nx, c.ny, c.pointX, c.pointY, fx, fy, bx, by, arcRadius);
                        if (res != null) {
                            c.hasArc = true; c.acx = (Double) res[0]; c.acy = (Double) res[1]; c.ar = arcRadius; c.frontIsEnd = frontIsEnd;
                            double[] span = arcSpan(c.nx, c.ny, (boolean[]) res[2], c.acx, c.acy);
                            c.hasSpan = true; c.span0 = span[0]; c.span1 = span[1];
                            break;
                        }
                    }
                }
                conns.add(c);
            }
        }
        return conns;
    }

    static Map<Long, Integer> spotIds() {
        Map<Long, Integer> ids = new HashMap<>();
        for (int i = 0; i < spotX.length; i++) ids.put(key((int) spotX[i], spotY[i]), i + 1);
        return ids;
    }

    static long key(int x, int y) { return ((long) x << 32) | (y & 0xffffffffL); }

    static int[][] cSecBufs = {new int[0], new int[0]};
    static int cSecToggle;

    static int[] sectionClumps(int[] clumps, List<Conn> conns) {
        if (cSecBufs[cSecToggle].length != H * W) cSecBufs[cSecToggle] = new int[H * W]; else Arrays.fill(cSecBufs[cSecToggle], 0);
        int[] sections = cSecBufs[cSecToggle];
        cSecToggle ^= 1;
        if (spotX.length == 0) return sections;
        Map<Long, Integer> ids = spotIds();
        int nc = conns.size();
        double[][] cuts = new double[nc][];
        for (int k = 0; k < nc; k++) {
            Conn c = conns.get(k);
            int startId = ids.get(key((int) c.sx, (int) c.sy)), endId = ids.get(key((int) c.ex, (int) c.ey));
            if (c.hasArc) {
                int front = c.frontIsEnd ? endId : startId, back = c.frontIsEnd ? startId : endId;
                cuts[k] = new double[]{0, front, back, c.acx, c.acy, c.ar, c.pointX - c.acx, c.pointY - c.acy};
            } else cuts[k] = new double[]{1, startId, endId, c.pointX, c.pointY, 0, c.ex - c.sx, c.ey - c.sy};
        }
        int most = 0;
        for (int k = 0; k < insideCount; k++) most = Math.max(most, clumps[insideIdx[k]]);
        int[] spotClump = new int[spotX.length];
        boolean[] hasSpot = new boolean[most + 1];
        for (int i = 0; i < spotX.length; i++) { spotClump[i] = clumps[spotY[i] * W + spotX[i]]; hasSpot[spotClump[i]] = true; }
        int[] starts = new int[most + 2];
        for (int k = 0; k < insideCount; k++) { int v = clumps[insideIdx[k]]; if (v > 0) starts[v + 1]++; }
        for (int c = 0; c <= most; c++) starts[c + 1] += starts[c];
        int[] pix = new int[starts[most + 1]];
        int[] fill = starts.clone();
        for (int k = 0; k < insideCount; k++) { int i = insideIdx[k]; if (clumps[i] > 0) pix[fill[clumps[i]]++] = i; }
        long[] version = new long[spotX.length + 1];
        long[] checkedFirst = new long[nc], checkedSecond = new long[nc];
        Arrays.fill(checkedFirst, -1);
        Arrays.fill(checkedSecond, -1);
        for (int clump = 1; clump <= most; clump++) {
            if (!hasSpot[clump]) continue;
            int a = starts[clump], b = starts[clump + 1];
            int[] owner = new int[b - a];
            int[] mem = new int[spotX.length];
            int memCount = 0;
            for (int s = 0; s < spotX.length; s++) if (spotClump[s] == clump) mem[memCount++] = s;
            for (int p = a; p < b; p++) {
                int y = pix[p] / W, x = pix[p] - y * W;
                long best = -1;
                for (int m = 0; m < memCount; m++) { int s = mem[m]; long ax = x - spotX[s], ay = y - spotY[s]; long d = ax * ax + ay * ay; if (best < 0 || d < best) { best = d; owner[p - a] = s + 1; } }
            }
            int[][] members = new int[spotX.length + 1][];
            int[] sizes = new int[spotX.length + 1];
            for (int p = a; p < b; p++) sizes[owner[p - a]]++;
            for (int m = 0; m < memCount; m++) members[mem[m] + 1] = new int[sizes[mem[m] + 1]];
            int[] put = new int[spotX.length + 1];
            for (int p = a; p < b; p++) { int o = owner[p - a]; members[o][put[o]++] = pix[p]; }
            for (int round = 0; round < memCount + 1; round++) {
                boolean moved = false;
                for (int k = 0; k < nc; k++) {
                    int first = (int) cuts[k][1], second = (int) cuts[k][2];
                    if (spotClump[first - 1] != clump) continue;
                    if (checkedFirst[k] == version[first] && checkedSecond[k] == version[second]) continue;
                    int[] fm = members[first] == null ? new int[0] : members[first], sm = members[second] == null ? new int[0] : members[second];
                    boolean[] goSecond = new boolean[fm.length], goFirst = new boolean[sm.length];
                    int nGoSecond = 0, nGoFirst = 0;
                    for (int q = 0; q < fm.length; q++) if (cutSide(cuts[k], fm[q], true)) { goSecond[q] = true; nGoSecond++; }
                    for (int q = 0; q < sm.length; q++) if (cutSide(cuts[k], sm[q], false)) { goFirst[q] = true; nGoFirst++; }
                    if (nGoSecond > 0 || nGoFirst > 0) {
                        moved = true;
                        int[] nf = new int[fm.length - nGoSecond + nGoFirst], ns = new int[sm.length - nGoFirst + nGoSecond];
                        int i1 = 0, i2 = 0;
                        for (int q = 0; q < fm.length; q++) if (!goSecond[q]) nf[i1++] = fm[q];
                        for (int q = 0; q < sm.length; q++) if (goFirst[q]) nf[i1++] = sm[q];
                        for (int q = 0; q < sm.length; q++) if (!goFirst[q]) ns[i2++] = sm[q];
                        for (int q = 0; q < fm.length; q++) if (goSecond[q]) ns[i2++] = fm[q];
                        members[first] = nf; members[second] = ns;
                        version[first]++; version[second]++;
                    } else { checkedFirst[k] = version[first]; checkedSecond[k] = version[second]; }
                }
                if (!moved) break;
            }
            for (int m = 0; m < memCount; m++) { int id = mem[m] + 1; if (members[id] != null) for (int px : members[id]) sections[px] = id; }
        }
        return sections;
    }

    static boolean cutSide(double[] cut, int pixel, boolean secondSide) {
        int py = pixel / W;
        double ax = (pixel - py * W) - cut[3], ay = py - cut[4];
        if (cut[0] == 0) {
            boolean within = dist(ax, ay) < cut[5];
            return secondSide ? !within && ax * cut[6] + ay * cut[7] > 0 : within;
        }
        double side = ax * cut[6] + ay * cut[7];
        return secondSide ? side > 0 : side < 0;
    }

    static byte[] cEdgeBuf = new byte[0];

    // Per-label raw (pre-fix) fit data cached between consecutive clumpCircles calls of one frame.
    static final class RRaw {
        double[] px, py, res;
        int area, outer;
        double mx, my;
        double fillRaw = Double.NaN, memoWant = Double.NaN, fillFixed, fixedScore = Double.NaN;
        double[] memoC;
    }
    static RRaw[] rRaws = new RRaw[0];
    static int[] rCacheLab = new int[0], rCacheIdx;
    static int rCacheN = -1, rCacheMost;
    static boolean rCacheOk;

    static List<Fit> clumpCircles(int[] labels, boolean[] inside, byte[] filled) { return rClump(labels, inside, filled, false); }

    // Only valid directly after another clumpCircles/rClumpCirclesInc call of the same frame (labels differ from that call's labels only in a few pixels).
    static List<Fit> rClumpCirclesInc(int[] labels, boolean[] inside, byte[] filled) { return rClump(labels, inside, filled, true); }

    static List<Fit> rClump(int[] labels, boolean[] inside, byte[] filled, boolean useCache) {
        int n = insideCount;
        int[] idx = insideIdx;
        boolean inc = useCache && rCacheOk && rCacheIdx == idx && rCacheN == n && rCacheLab.length >= n;
        rCacheOk = false;
        if (rCacheLab.length < n) rCacheLab = new int[n];
        int[] cacheLab = rCacheLab;
        if (cEdgeBuf.length < n) cEdgeBuf = new byte[n];
        byte[] edge = cEdgeBuf;
        Arrays.fill(edge, 0, n, (byte) 0);
        int most = 0;
        for (int k = 0; k < n; k++) { int l = labels[idx[k]]; if (l > most) most = l; }
        final boolean[] dirty = new boolean[Math.max(most, inc ? rCacheMost : 0) + 1];
        if (!inc) { Arrays.fill(dirty, true); for (int k = 0; k < n; k++) cacheLab[k] = labels[idx[k]]; }
        else for (int k = 0; k < n; k++) {
            int a = labels[idx[k]], b = cacheLab[k];
            if (a != b) { if (a > 0) dirty[a] = true; if (b > 0) dirty[b] = true; cacheLab[k] = a; }
        }
        for (int k = 0; k < n; k++) {
            int i = idx[k], label = labels[i];
            if (label <= 0 || !dirty[label]) continue;
            if (isOuter(inside, i)) edge[k] = 1;
            else if (labels[i - W] != label || labels[i + W] != label || labels[i - 1] != label || labels[i + 1] != label) edge[k] = 2;
        }
        int[] area = new int[most + 1], edgeCount = new int[most + 1], outerCount = new int[most + 1];
        double[] sumX = new double[most + 1], sumY = new double[most + 1];
        for (int k = 0; k < n; k++) {
            int i = idx[k], l = labels[i];
            if (l <= 0 || !dirty[l]) continue;
            int iy = i / W;
            area[l]++; sumX[l] += i - iy * W; sumY[l] += iy;
            if (edge[k] == 1) { edgeCount[l]++; outerCount[l]++; } else if (edge[k] == 2) edgeCount[l]++;
        }
        double[][] px = new double[most + 1][], py = new double[most + 1][];
        int[] fillCount = new int[most + 1];
        for (int l = 1; l <= most; l++) if (dirty[l] && area[l] >= regionMinPx) { px[l] = new double[edgeCount[l]]; py[l] = new double[edgeCount[l]]; fillCount[l] = outerCount[l]; }
        int[] outerFill = new int[most + 1];
        for (int k = 0; k < n; k++) {
            int e = edge[k];
            if (e == 0) continue;
            int i = idx[k], l = labels[i];
            if (l <= 0 || px[l] == null) continue;
            int iy = i / W;
            int at = e == 1 ? outerFill[l]++ : fillCount[l]++;
            px[l][at] = i - iy * W; py[l][at] = iy;
        }
        final RRaw[] raws = new RRaw[most + 1];
        List<Runnable> jobs = new ArrayList<>();
        for (int label = 1; label <= most; label++) {
            final int l = label;
            if (!dirty[l]) { if (inc && l < rRaws.length) raws[l] = rRaws[l]; continue; }
            if (px[l] == null) continue;
            final RRaw rw = new RRaw();
            rw.px = px[l]; rw.py = py[l]; rw.area = area[l]; rw.outer = outerCount[l];
            rw.mx = sumX[l] / area[l]; rw.my = sumY[l] / area[l];
            raws[l] = rw;
            if (px[l].length < 5) continue;
            final int outerN = outerCount[l];
            jobs.add(() -> {
                double[] w = new double[rw.px.length];
                Arrays.fill(w, 1.0);
                if (outerN >= edgeMinPoints) for (int k = outerN; k < w.length; k++) w[k] = seamWeight;
                rw.res = fitRegion(rw.px, rw.py, w);
            });
        }
        runAll(jobs);
        List<Fit> fits = new ArrayList<>();
        List<RRaw> fitRawList = new ArrayList<>();
        for (int l = 1; l <= most; l++) {
            RRaw rw = raws[l];
            if (rw == null || rw.res == null) continue;
            double[] res = rw.res;
            Fit f = new Fit();
            f.cx = res[0]; f.cy = res[1]; f.r = res[2]; f.score = res[3]; f.px = rw.px; f.py = rw.py; f.area = rw.area; f.outer = rw.outer;
            f.mx = rw.mx; f.my = rw.my; f.label = l;
            fits.add(f);
            fitRawList.add(rw);
        }
        List<double[]> refs = new ArrayList<>();
        for (double[] c : accepted) refs.add(new double[]{c[1], c[2]});
        for (Fit f : fits) if (f.score >= fitMinPctg && f.outer >= edgeMinPoints) refs.add(new double[]{f.cy, f.r});
        Expect expected = radiusByRow(refs);
        final boolean[] fillOk = new boolean[fits.size()];
        List<Runnable> fixJobs = new ArrayList<>();
        for (int fi = 0; fi < fits.size(); fi++) {
            final Fit f = fits.get(fi);
            final RRaw rw = fitRawList.get(fi);
            final int at = fi;
            fixJobs.add(() -> {
                boolean useFixed = false;
                if (expected != null) {
                    double want = Math.max(expected.at(f.cy), Math.sqrt(f.area / Math.PI));
                    if (Math.abs(f.r - want) > radiusTolerancePctg * want) {
                        if (!(rw.memoWant == want)) {
                            double[] c0 = fixedRadiusCenter(f.px, f.py, null, f.mx, f.my, want);
                            rw.memoC = c0;
                            rw.fillFixed = circleFill(filled, c0[0], c0[1], want);
                            rw.fixedScore = Double.NaN;
                            rw.memoWant = want;
                        }
                        double[] c = rw.memoC;
                        if (Double.isNaN(rw.fillRaw)) rw.fillRaw = circleFill(filled, f.cx, f.cy, f.r);
                        if (rw.fillFixed >= circleMinFillPctg || rw.fillRaw < circleMinFillPctg) {
                            f.cx = c[0]; f.cy = c[1]; f.r = want;
                            if (Double.isNaN(rw.fixedScore)) {
                                boolean[] keep = new boolean[f.px.length];
                                rw.fixedScore = (double) markOnCircle(f.px, f.py, f.cx, f.cy, f.r, keep) / f.px.length;
                            }
                            f.score = rw.fixedScore;
                            f.fixed = true;
                            useFixed = true;
                        } else if (Math.abs(f.r - want) > radiusRejectPctg * want) f.drop = true;
                    }
                }
                if (f.drop) return;
                double fill;
                if (useFixed) fill = rw.fillFixed;
                else { if (Double.isNaN(rw.fillRaw)) rw.fillRaw = circleFill(filled, f.cx, f.cy, f.r); fill = rw.fillRaw; }
                fillOk[at] = fill >= circleMinFillPctg;
            });
        }
        runAll(fixJobs);
        rRaws = raws;
        rCacheMost = most;
        rCacheIdx = idx;
        rCacheN = n;
        rCacheOk = true;
        List<Fit> good = new ArrayList<>();
        for (int fi = 0; fi < fits.size(); fi++) if (fillOk[fi]) good.add(fits.get(fi));
        good.sort((a, b) -> a.score != b.score ? Double.compare(b.score, a.score) : Integer.compare(b.area, a.area));
        List<Fit> kept = new ArrayList<>();
        for (Fit f : good) {
            boolean crowded = false;
            for (Fit o : kept) if (dist(f.cx - o.cx, f.cy - o.cy) < duplicateCenterPctg * Math.min(f.r, o.r)) { crowded = true; break; }
            if (!crowded) kept.add(f);
        }
        return kept;
    }

    static Object[] outlineCircle(double[] rx, double[] ry, double spotXv, double spotYv, Double want) {
        if (rx.length < arcMinPoints || want == null) return null;
        double cx = spotXv, cy = spotYv + want / 2;
        boolean[] keep = new boolean[rx.length];
        Arrays.fill(keep, true);
        int kept = rx.length;
        for (int it = 0; it < fitRefineRounds + 2; it++) {
            if (kept < arcMinPoints) return null;
            double[] c = fixedRadiusCenter(rx, ry, keep, cx, cy, want);
            cx = c[0]; cy = c[1];
            kept = markOnCircle(rx, ry, cx, cy, want, keep);
        }
        if (dist(spotXv - cx, spotYv - cy) >= want) return null;
        boolean[] seen = new boolean[37];
        int bins = 0;
        for (int i = 0; i < rx.length; i++) if (keep[i]) {
            double angle = Math.toDegrees(Math.atan2(ry[i] - cy, rx[i] - cx)) % 360;
            if (angle < 0) angle += 360;
            int bin = (int) Math.floor(angle / 10);
            if (!seen[bin]) { seen[bin] = true; bins++; }
        }
        if (bins / 36.0 < outlineMinPctg) return null;
        return new Object[]{cx, cy, (double) want};
    }

    static int[] eRimList = new int[0];

    static void ballArcs(List<Conn> conns, int[] firstSections, boolean[] inside, Expect expected) {
        Map<Long, Integer> ids = spotIds();
        // rim pixels grouped by section id (counting sort; ascending insideIdx order kept within each section)
        if (eRimList.length < insideCount) eRimList = new int[insideCount];
        int nRim = 0, most = 0;
        for (int k = 0; k < insideCount; k++) { int i = insideIdx[k]; if (firstSections[i] > 0 && isOuter(inside, i)) { eRimList[nRim++] = i; most = Math.max(most, firstSections[i]); } }
        int[] rimStart = new int[most + 2];
        for (int r = 0; r < nRim; r++) rimStart[firstSections[eRimList[r]] + 1]++;
        for (int c = 0; c <= most; c++) rimStart[c + 1] += rimStart[c];
        int[] rimPixArr = new int[nRim];
        int[] rimFill = rimStart.clone();
        for (int r = 0; r < nRim; r++) { int i = eRimList[r]; rimPixArr[rimFill[firstSections[i]]++] = i; }
        Map<Long, double[][]> rims = new HashMap<>();
        Map<Long, double[]> outlineCircles = new HashMap<>();
        for (int s = 0; s < spotX.length; s++) {
            long k = key(spotX[s], spotY[s]);
            boolean known = s + 1 <= most;
            int rFrom = known ? rimStart[s + 1] : 0, np = known ? rimStart[s + 2] - rFrom : 0;
            double[] rx = new double[np], ry = new double[np];
            for (int i = 0; i < np; i++) { int px = rimPixArr[rFrom + i]; rx[i] = px % W; ry[i] = px / W; }
            rims.put(k, new double[][]{rx, ry});
            Object[] c = outlineCircle(rx, ry, spotX[s], spotY[s], expected != null ? expected.at(spotY[s]) : null);
            if (c != null) outlineCircles.put(k, new double[]{(Double) c[0], (Double) c[1], (Double) c[2]});
        }
        for (Conn c : conns) {
            if (c.nx.length < arcMinPoints) continue;
            double bestMiss = 0, bcx = 0, bcy = 0, br = 0;
            boolean found = false, bestFront = false;
            for (boolean frontIsEnd : new boolean[]{true, false}) {
                double fx = frontIsEnd ? c.ex : c.sx, fy = frontIsEnd ? c.ey : c.sy, bx = frontIsEnd ? c.sx : c.ex, by = frontIsEnd ? c.sy : c.ey;
                double[] circle = outlineCircles.get(key((int) fx, (int) fy));
                if (circle == null) continue;
                if (dist(fx - circle[0], fy - circle[1]) >= circle[2] || dist(bx - circle[0], by - circle[1]) <= circle[2]) continue;
                double[] d = new double[c.nx.length];
                for (int i = 0; i < d.length; i++) d[i] = Math.abs(dist(c.nx[i] - circle[0], c.ny[i] - circle[1]) - circle[2]);
                double miss = median(d, d.length);
                if (miss <= outlineFitMaxPctg * circle[2] && (!found || miss < bestMiss)) { found = true; bestMiss = miss; bestFront = frontIsEnd; bcx = circle[0]; bcy = circle[1]; br = circle[2]; }
            }
            if (found) { c.hasArc = true; c.acx = bcx; c.acy = bcy; c.ar = br; c.frontIsEnd = bestFront; }
        }
        LinkedHashMap<Long, List<Conn>> groups = new LinkedHashMap<>();
        for (Conn c : conns) if (c.hasArc) groups.computeIfAbsent(c.frontIsEnd ? key((int) c.ex, (int) c.ey) : key((int) c.sx, (int) c.sy), z -> new ArrayList<>()).add(c);
        Map<Long, double[]> arcsByFront = new HashMap<>();
        for (Map.Entry<Long, List<Conn>> e : groups.entrySet()) {
            long front = e.getKey();
            List<Conn> members = e.getValue();
            int crease = 0;
            for (Conn m : members) crease += m.nx.length;
            double[][] rim = rims.get(front);
            double[] cx = new double[crease], cy = new double[crease], ax = new double[crease + rim[0].length], ay = new double[crease + rim[0].length];
            int k = 0;
            for (Conn m : members) for (int i = 0; i < m.nx.length; i++) { cx[k] = m.nx[i]; cy[k] = m.ny[i]; ax[k] = m.nx[i]; ay[k] = m.ny[i]; k++; }
            for (int i = 0; i < rim[0].length; i++) { ax[k] = rim[0][i]; ay[k] = rim[1][i]; k++; }
            double centerX, centerY, r;
            double[] oc = outlineCircles.get(front);
            if (oc != null) { centerX = oc[0]; centerY = oc[1]; r = oc[2]; }
            else {
                r = members.get(0).ar;
                double sx = 0, sy = 0;
                for (Conn m : members) { sx += m.acx; sy += m.acy; }
                centerX = sx / members.size(); centerY = sy / members.size();
            }
            boolean[] keep = new boolean[ax.length];
            Arrays.fill(keep, true);
            int kept = ax.length;
            for (int it = 0; it < fitRefineRounds + 2; it++) {
                if (oc != null || kept < arcMinPoints) break;
                double[] c = fixedRadiusCenter(ax, ay, keep, centerX, centerY, r);
                centerX = c[0]; centerY = c[1];
                kept = markOnCircle(ax, ay, centerX, centerY, r, keep);
            }
            boolean[] on = new boolean[crease];
            if (markOnCircle(cx, cy, centerX, centerY, r, on) < 2) continue;
            double[] span = arcSpan(cx, cy, on, centerX, centerY);
            for (Conn m : members) { m.acx = centerX; m.acy = centerY; m.ar = r; m.hasSpan = true; m.span0 = span[0]; m.span1 = span[1]; }
            arcsByFront.put(front, new double[]{centerX, centerY, r, span[0], span[1]});
        }
        for (Conn c : conns) {
            if (c.hasArc) continue;
            boolean lowerIsEnd = c.ey >= c.sy;
            double lx = lowerIsEnd ? c.ex : c.sx, ly = lowerIsEnd ? c.ey : c.sy, ux = lowerIsEnd ? c.sx : c.ex, uy = lowerIsEnd ? c.sy : c.ey;
            double[] f = arcsByFront.get(key((int) lx, (int) ly));
            if (f == null) continue;
            if (dist(ux - f[0], uy - f[1]) <= f[2]) continue;
            c.hasArc = true; c.acx = f[0]; c.acy = f[1]; c.ar = f[2]; c.frontIsEnd = lowerIsEnd; c.hasSpan = true; c.span0 = f[3]; c.span1 = f[4];
        }
    }

    static double[] linspace(double a, double b, int n) {
        double[] out = new double[n];
        if (n == 1) { out[0] = a; return out; }
        double step = (b - a) / (n - 1);
        for (int i = 0; i < n; i++) out[i] = a + i * step;
        out[n - 1] = b;
        return out;
    }

    static double[] shiftFit(double[][] model, double[] offsets, int rounds) {
        int n = offsets.length, cols = model[0].length;
        boolean[] keep = new boolean[n];
        Arrays.fill(keep, true);
        double[] solution = new double[cols];
        for (int it = 0; it < rounds; it++) {
            int k = 0;
            for (boolean b : keep) if (b) k++;
            if (k < arcMinPoints) break;
            double[][] a = new double[k][];
            double[] y = new double[k];
            int j = 0;
            for (int i = 0; i < n; i++) if (keep[i]) { a[j] = model[i]; y[j] = offsets[i]; j++; }
            solution = lstsq(a, y);
            double[] residual = new double[n];
            for (int i = 0; i < n; i++) { double v = -offsets[i]; for (int c = 0; c < cols; c++) v += model[i][c] * solution[c]; residual[i] = Math.abs(v); }
            double[] kr = new double[k];
            j = 0;
            for (int i = 0; i < n; i++) if (keep[i]) kr[j++] = residual[i];
            double limit = Math.max(median(kr, k) * 2, 1.0);
            for (int i = 0; i < n; i++) keep[i] = residual[i] <= limit;
        }
        return solution;
    }

    static List<double[]> shiftArcs(List<Conn> conns, byte[] brightness, boolean[] inside) {
        LinkedHashMap<String, List<Conn>> groups = new LinkedHashMap<>();
        for (Conn c : conns) if (c.hasArc && c.hasSpan) groups.computeIfAbsent(Math.rint(c.acx * 1000) + "," + Math.rint(c.acy * 1000) + "," + Math.rint(c.ar * 1000), z -> new ArrayList<>()).add(c);
        List<double[]> shifted = new ArrayList<>();
        for (List<Conn> members : groups.values()) {
            Conn first = members.get(0);
            double cx = first.acx, cy = first.acy, r = first.ar, s0 = first.span0, s1 = first.span1;
            int reach = Math.max((int) Math.rint(arcShiftPctg * r), 1);
            int count = Math.max((int) (r * Math.toRadians(s1 - s0)) + 1, 4);
            double[] angles = linspace(s0, s1, count);
            for (int i = 0; i < count; i++) angles[i] = Math.toRadians(angles[i]);
            int span = 2 * reach + 1;
            double[][] values = new double[count][span];
            for (int i = 0; i < count; i++) for (int o = 0; o < span; o++) {
                double rr = r + (o - reach);
                int qx = Math.min(Math.max((int) Math.rint(cx + rr * Math.cos(angles[i])), 0), W - 1), qy = Math.min(Math.max((int) Math.rint(cy + rr * Math.sin(angles[i])), 0), H - 1);
                values[i][o] = inside[qy * W + qx] ? (brightness[qy * W + qx] & 255) : Double.POSITIVE_INFINITY;
            }
            List<Double> va = new ArrayList<>(), vo = new ArrayList<>();
            for (int i = 0; i < count; i++) {
                int best = 0;
                double bestV = Double.NaN;
                for (int o = 0; o < span; o++) {
                    double sm = i > 0 && i < count - 1 && count >= 3 ? (values[i - 1][o] + values[i][o] + values[i + 1][o]) / 3 : values[i][o];
                    if (o == 0 || sm < bestV) { bestV = sm; best = o; }
                }
                if (!Double.isInfinite(bestV)) { va.add(angles[i]); vo.add((double) (best - reach)); }
            }
            double ncx = cx, ncy = cy;
            if (va.size() >= arcMinPoints) {
                boolean full = s1 - s0 >= arcShiftFullDeg;
                double middle = Math.toRadians((s0 + s1) / 2);
                double[][] model = new double[va.size()][];
                double[] off = new double[va.size()];
                for (int i = 0; i < va.size(); i++) { double a = va.get(i); model[i] = full ? new double[]{Math.cos(a), Math.sin(a)} : new double[]{Math.cos(a - middle)}; off[i] = vo.get(i); }
                double[] sol = shiftFit(model, off, fitRefineRounds + 2);
                double mx = full ? sol[0] : sol[0] * Math.cos(middle), my = full ? sol[1] : sol[0] * Math.sin(middle);
                if (dist(mx, my) <= arcShiftPctg * r) { ncx = cx + mx; ncy = cy + my; }
            }
            shifted.add(new double[]{ncx, ncy, r, s0, s1});
            for (Conn m : members) { m.acx = ncx; m.acy = ncy; }
        }
        return shifted;
    }

    static int[] dGrowPainted = new int[0];
    static int[] dGrowTouched = new int[256];
    static int dGrowTouchedCount;

    static void dGrowPaint(int idx, int value) {
        dGrowPainted[idx] = value;
        if (dGrowTouchedCount == dGrowTouched.length) dGrowTouched = Arrays.copyOf(dGrowTouched, 2 * dGrowTouchedCount);
        dGrowTouched[dGrowTouchedCount++] = idx;
    }

    static double[][] growArcs(List<double[]> arcs, boolean[] inside, boolean[][] frozen, boolean[][] cut) {
        int count = arcs.size();
        if (dGrowPainted.length != H * W) dGrowPainted = new int[H * W];
        int[] painted = dGrowPainted;
        dGrowTouchedCount = 0;
        double[][] grown = new double[count][2];
        double[] steps = new double[count];
        boolean[][] running = new boolean[count][2];
        int[][] active = new int[2 * count][2];
        for (int i = 0; i < count; i++) {
            double[] a = arcs.get(i);
            double middle = (a[3] + a[4]) / 2;
            int x = (int) Math.rint(a[0] + a[2] * Math.cos(Math.toRadians(middle))), y = (int) Math.rint(a[1] + a[2] * Math.sin(Math.toRadians(middle)));
            if (x >= 0 && x < W && y >= 0 && y < H) dGrowPaint(y * W + x, i + 1);
            grown[i][0] = middle; grown[i][1] = middle;
            steps[i] = Math.toDegrees(1 / Math.max(a[2], 1));
            active[2 * i] = new int[]{i, 1}; active[2 * i + 1] = new int[]{i, 0};
            running[i][0] = running[i][1] = true;
        }
        int total = 2 * count;
        int[][] still = new int[2 * count][2];
        while (total > 0) {
            int kept = 0;
            for (int n = 0; n < total; n++) {
                int i = active[n][0], side = active[n][1];
                if (!running[i][side]) continue;
                double[] a = arcs.get(i);
                double step = side == 1 ? steps[i] : -steps[i];
                if (grown[i][1] - grown[i][0] + Math.abs(step) >= 360) {
                    for (int o = 0; o < 2; o++) if (running[i][o]) { cut[i][o] = true; running[i][o] = false; if (o == 1) grown[i][1] = Math.min(grown[i][1], a[4]); else grown[i][0] = Math.max(grown[i][0], a[3]); }
                    continue;
                }
                double angle = grown[i][side] + step;
                if (frozen[i][side] && (side == 1 ? angle > a[4] : angle < a[3])) { running[i][side] = false; continue; }
                int x = (int) Math.rint(a[0] + a[2] * Math.cos(Math.toRadians(angle))), y = (int) Math.rint(a[1] + a[2] * Math.sin(Math.toRadians(angle)));
                if (x < 0 || x >= W || y < 0 || y >= H || !inside[y * W + x]) { running[i][side] = false; continue; }
                boolean blocked = false;
                for (int yy = y - 1; yy <= y + 1 && !blocked; yy++) for (int xx = x - 1; xx <= x + 1; xx++) if (yy >= 0 && yy < H && xx >= 0 && xx < W && painted[yy * W + xx] != 0 && painted[yy * W + xx] != i + 1) { blocked = true; break; }
                if (blocked) { running[i][side] = false; continue; }
                dGrowPaint(y * W + x, i + 1);
                grown[i][side] = angle;
                still[kept][0] = i; still[kept][1] = side; kept++;
            }
            int[][] t = active; active = still; still = t;
            total = kept;
        }
        for (int t = 0; t < dGrowTouchedCount; t++) painted[dGrowTouched[t]] = 0;
        dGrowTouchedCount = 0;
        return grown;
    }

    static double[][] extendArcs(List<double[]> arcs, boolean[] inside) {
        int count = arcs.size();
        boolean[][] frozen = new boolean[count][2];
        while (true) {
            boolean[][] cut = new boolean[count][2];
            double[][] grown = growArcs(arcs, inside, frozen, cut);
            boolean fresh = false;
            for (int i = 0; i < count; i++) for (int s = 0; s < 2; s++) if (cut[i][s] && !frozen[i][s]) fresh = true;
            if (!fresh) return grown;
            for (int i = 0; i < count; i++) for (int s = 0; s < 2; s++) frozen[i][s] |= cut[i][s];
        }
    }

    static List<double[][]> snapArcs(List<double[]> arcs, double[][] grown, boolean[] inside, byte[] insideBytes) {
        List<MatOfPoint> edges = new ArrayList<>();
        long s = tic();
        findContoursTracked(bytesMat(insideBytes, H, W), edges, tmp(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        toc(s);
        int total = 0;
        for (MatOfPoint e : edges) total += (int) e.total();
        double[] ox = new double[total], oy = new double[total];
        int k = 0;
        int minX = W, minY = H, maxX = -1, maxY = -1;
        for (MatOfPoint e : edges) {
            int[] p = new int[(int) e.total() * 2];
            e.get(0, 0, p);
            for (int i = 0; i < p.length / 2; i++) {
                ox[k] = p[2 * i]; oy[k] = p[2 * i + 1]; k++;
                minX = Math.min(minX, p[2 * i]); maxX = Math.max(maxX, p[2 * i]);
                minY = Math.min(minY, p[2 * i + 1]); maxY = Math.max(maxY, p[2 * i + 1]);
            }
        }
        int count = arcs.size();
        // The distance map is only ever compared with meet (at most ~1.05 * meet along a chamfer path), so it is
        // computed on a crop that holds every outline pixel plus a margin beyond any meet. Points outside the crop
        // are farther than meet from the outline and are treated as such. With no outline the full frame is used.
        double maxMeet = 1.5;
        for (int i = 0; i < count; i++) maxMeet = Math.max(maxMeet, arcMeetPctg * arcs.get(i)[2]);
        int x0 = 0, y0 = 0, x1 = W, y1 = H;
        if (total > 0) {
            int margin = (int) Math.ceil(1.1 * maxMeet) + 2;
            x0 = Math.max(minX - margin, 0); y0 = Math.max(minY - margin, 0);
            x1 = Math.min(maxX + margin + 1, W); y1 = Math.min(maxY + margin + 1, H);
        }
        int cw = x1 - x0, ch = y1 - y0;
        byte[] crop = new byte[cw * ch];
        Arrays.fill(crop, (byte) 255);
        for (int q = 0; q < total; q++) crop[((int) oy[q] - y0) * cw + (int) ox[q] - x0] = 0;
        Mat cm = bytesMat(crop, ch, cw), dist = tmp();
        s = tic();
        Imgproc.distanceTransform(cm, dist, Imgproc.DIST_L2, 3);
        toc(s);
        float[] toOutline = getFloats(dist);
        List<double[][]> paths = new ArrayList<>();
        if (dSnapPainted.length != H * W) dSnapPainted = new int[H * W];
        int[] painted = dSnapPainted;
        for (int i = 0; i < count; i++) {
            double[] a = arcs.get(i);
            int n = Math.max((int) (a[2] * Math.toRadians(grown[i][1] - grown[i][0])) + 1, 8);
            double[] ang = linspace(grown[i][0], grown[i][1], n);
            double[] xs = new double[n], ys = new double[n];
            for (int j = 0; j < n; j++) { double t = Math.toRadians(ang[j]); xs[j] = a[0] + a[2] * Math.cos(t); ys[j] = a[1] + a[2] * Math.sin(t); painted[clampY((int) Math.rint(ys[j])) * W + clampX((int) Math.rint(xs[j]))] = i + 1; }
            paths.add(new double[][]{xs, ys});
        }
        List<double[][]> snapped = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double[] a = arcs.get(i);
            double[] xs = paths.get(i)[0], ys = paths.get(i)[1];
            int n = xs.length;
            double[] angles = linspace(grown[i][0], grown[i][1], n);
            int firstOriginal = -1, lastOriginal = -1;
            for (int j = 0; j < n; j++) if (angles[j] >= a[3] - 1e-6 && angles[j] <= a[4] + 1e-6) { if (firstOriginal < 0) firstOriginal = j; lastOriginal = j; }
            if (firstOriginal < 0) { firstOriginal = 0; lastOriginal = n - 1; }
            double meet = Math.max(arcMeetPctg * a[2], 1.5);
            int keepFrom = 0, keepTo = n - 1;
            for (int j = lastOriginal; j < n; j++) if (dSnapNear(toOutline, x0, y0, cw, ch, xs[j], ys[j]) <= meet) { keepTo = j; break; }
            for (int j = firstOriginal; j >= 0; j--) if (dSnapNear(toOutline, x0, y0, cw, ch, xs[j], ys[j]) <= meet) { keepFrom = j; break; }
            double[] px = Arrays.copyOfRange(xs, keepFrom, keepTo + 1), py = Arrays.copyOfRange(ys, keepFrom, keepTo + 1);
            int m = px.length;
            int bend = Math.min(Math.max((int) (arcBendPctg * a[2]), 2), m);
            double[] ramp = linspace(0, 1, bend);
            for (int j = 0; j < bend; j++) ramp[j] *= ramp[j];
            for (int end : new int[]{0, m - 1}) {
                int x = (int) Math.rint(px[end]), y = (int) Math.rint(py[end]);
                boolean other = false;
                for (int yy = Math.max(y - 1, 0); yy < Math.min(y + 2, H); yy++) for (int xx = Math.max(x - 1, 0); xx < Math.min(x + 2, W); xx++) if (painted[yy * W + xx] > 0 && painted[yy * W + xx] != i + 1) other = true;
                if (other || total == 0) continue;
                int nearest = 0;
                double nd = Double.MAX_VALUE;
                for (int q = 0; q < total; q++) { double d = dist(ox[q] - px[end], oy[q] - py[end]); if (d < nd) { nd = d; nearest = q; } }
                if (nd <= 1.5 || nd > arcSnapPctg * a[2]) continue;
                double offX = ox[nearest] - px[end], offY = oy[nearest] - py[end];
                if (end == 0) for (int j = 0; j < bend; j++) { px[j] += ramp[bend - 1 - j] * offX; py[j] += ramp[bend - 1 - j] * offY; }
                else for (int j = 0; j < bend; j++) { px[m - bend + j] += ramp[j] * offX; py[m - bend + j] += ramp[j] * offY; }
            }
            int bestStart = 0, bestLength = 0, start = -1;
            for (int j = 0; j <= m; j++) {
                boolean ok = j < m && inside[clampY((int) Math.rint(py[j])) * W + clampX((int) Math.rint(px[j]))];
                if (ok && start < 0) start = j;
                if (!ok && start >= 0) { if (j - start > bestLength) { bestStart = start; bestLength = j - start; } start = -1; }
            }
            snapped.add(new double[][]{Arrays.copyOfRange(px, bestStart, bestStart + bestLength), Arrays.copyOfRange(py, bestStart, bestStart + bestLength)});
        }
        for (double[][] p : paths) for (int j = 0; j < p[0].length; j++) painted[clampY((int) Math.rint(p[1][j])) * W + clampX((int) Math.rint(p[0][j]))] = 0;
        return snapped;
    }

    static int[] dSnapPainted = new int[0];

    static float dSnapNear(float[] d, int x0, int y0, int cw, int ch, double x, double y) {
        int px = clampX((int) Math.rint(x)), py = clampY((int) Math.rint(y));
        if (px < x0 || px >= x0 + cw || py < y0 || py >= y0 + ch) return Float.MAX_VALUE;
        return d[(py - y0) * cw + px - x0];
    }

    static int clampX(int x) { return Math.min(Math.max(x, 0), W - 1); }
    static int clampY(int y) { return Math.min(Math.max(y, 0), H - 1); }

    static int[] cPieceOut = new int[0];

    static int[] largestPieces(int[] sections) {
        if (cPieceOut.length != H * W) cPieceOut = new int[H * W]; else Arrays.fill(cPieceOut, 0);
        return largestPieces(sections, cPieceOut);
    }

    static int[] largestPieces(int[] sections, int[] cleaned) {
        if (pieceBuf.length < H * W) { pieceBuf = new int[H * W]; queueBuf = new int[H * W]; }
        int[] piece = pieceBuf, queue = queueBuf;
        int n = insideCount;
        int[] idx = insideIdx;
        int most = 0;
        for (int k = 0; k < n; k++) { int i = idx[k]; piece[i] = -1; if (sections[i] > most) most = sections[i]; }
        int[] pieceSize = new int[n + 1], pieceLabel = new int[n + 1];
        int count = 0;
        for (int k = 0; k < n; k++) {
            int i0 = idx[k];
            int label = sections[i0];
            if (label <= 0 || piece[i0] >= 0) continue;
            piece[i0] = count;
            queue[0] = i0;
            int head = 0, tail = 1;
            while (head < tail) {
                int at = queue[head++], ay = at / W, ax = at - ay * W;
                if (ay > 0 && ay < H - 1 && ax > 0 && ax < W - 1) {
                    int j = at - W - 1;
                    if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                    j++;
                    if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                    j++;
                    if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                    j = at - 1;
                    if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                    j += 2;
                    if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                    j = at + W - 1;
                    if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                    j++;
                    if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                    j++;
                    if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                    continue;
                }
                int y0 = ay > 0 ? ay - 1 : 0, y1 = ay + 2 < H ? ay + 2 : H, x0 = ax > 0 ? ax - 1 : 0, x1 = ax + 2 < W ? ax + 2 : W;
                for (int yy = y0; yy < y1; yy++) {
                    int row = yy * W;
                    for (int xx = x0; xx < x1; xx++) {
                        int j = row + xx;
                        if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                    }
                }
            }
            pieceSize[count] = tail;
            pieceLabel[count] = label;
            count++;
        }
        int[] bestPiece = new int[most + 1], bestSize = new int[most + 1];
        Arrays.fill(bestPiece, -1);
        for (int p = 0; p < count; p++) { int l = pieceLabel[p]; if (pieceSize[p] > bestSize[l]) { bestSize[l] = pieceSize[p]; bestPiece[l] = p; } }
        for (int k = 0; k < n; k++) { int i = idx[k]; int l = sections[i]; if (l > 0 && bestPiece[l] == piece[i]) cleaned[i] = l; }
        return cleaned;
    }

    static byte[] cWallBuf = new byte[0], cOpenBuf = new byte[0];
    static int[] cMergedBuf = new int[0];

    static int[] sectionByArcs(boolean[] inside, List<double[][]> paths, int[] fallback) {
        Mat walls = tmpZeros(H, W, CvType.CV_8UC1);
        long s = tic();
        for (double[][] p : paths) {
            int n = p[0].length;
            if (n >= 2) {
                Point[] pts = new Point[n];
                for (int j = 0; j < n; j++) pts[j] = new Point(Math.rint(p[0][j] * 16), Math.rint(p[1][j] * 16));
                Imgproc.polylines(walls, Collections.singletonList(tmpPoints(pts)), false, new Scalar(255), 1, Imgproc.LINE_8, 4);
            } else if (n == 1) walls.put((int) Math.rint(p[1][0]), (int) Math.rint(p[0][0]), 255);
        }
        toc(s);
        if (cWallBuf.length != H * W) { cWallBuf = new byte[H * W]; cOpenBuf = new byte[H * W]; }
        byte[] wb = cWallBuf;
        walls.get(0, 0, wb);
        byte[] open = cOpenBuf;
        for (int i = 0; i < H * W; i++) open[i] = (byte) (inside[i] && wb[i] == 0 ? 1 : 0);
        int[] cnt = new int[1];
        int[] pieces = connected(open, 4, cnt, 3);
        int count = cnt[0];
        int[] spotCount = new int[count], owner = new int[count];
        for (int sp = 0; sp < spotX.length; sp++) spotCount[pieces[spotY[sp] * W + spotX[sp]]]++;
        for (int sp = 0; sp < spotX.length; sp++) { int p = pieces[spotY[sp] * W + spotX[sp]]; if (p > 0 && spotCount[p] == 1) owner[p] = sp + 1; }
        if (cMergedBuf.length != H * W) cMergedBuf = new int[H * W]; else Arrays.fill(cMergedBuf, 0);
        int[] merged = cMergedBuf;
        for (int k = 0; k < insideCount; k++) { int i = insideIdx[k]; int o = owner[pieces[i]]; merged[i] = o > 0 ? o : fallback[i]; }
        return largestPieces(merged);
    }

    static Map<Long, Integer> frontPairs(List<Conn> conns) {
        Map<Long, Integer> ids = spotIds();
        Map<Long, Integer> fronts = new HashMap<>();
        for (Conn c : conns) {
            if (!c.hasArc) continue;
            int a = ids.get(key((int) c.sx, (int) c.sy)), b = ids.get(key((int) c.ex, (int) c.ey));
            fronts.put(key(Math.min(a, b), Math.max(a, b)), c.frontIsEnd ? b : a);
        }
        return fronts;
    }

    static int[] assignBuf = new int[0];
    static int[][] refitBufs = {new int[0], new int[0]};

    static int[] assignToCircles(int[] clumps, int clumpCount, boolean[] inside, List<Fit> fits, Map<Long, Integer> fronts) {
        if (assignBuf.length != H * W) assignBuf = new int[H * W];
        int[] sections = assignBuf;
        for (int k = 0; k < insideCount; k++) sections[insideIdx[k]] = 0;
        if (fits.isEmpty()) return sections;
        int n = fits.size();
        int[] circleClump = new int[n];
        for (int i = 0; i < n; i++) { Fit f = fits.get(i); circleClump[i] = clumps[Math.min(Math.max((int) Math.rint(f.my), 0), H - 1) * W + Math.min(Math.max((int) Math.rint(f.mx), 0), W - 1)]; }
        int[][] known = new int[n][n];
        for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) { int a = fits.get(i).label, b = fits.get(j).label; Integer v = fronts.get(key(Math.min(a, b), Math.max(a, b))); known[i][j] = v == null ? -1 : v; }
        int[] starts = new int[clumpCount + 1];
        for (int i = 0; i < n; i++) starts[circleClump[i] + 1]++;
        for (int c = 0; c < clumpCount; c++) starts[c + 1] += starts[c];
        int[] members = new int[n], fill = starts.clone();
        for (int i = 0; i < n; i++) members[fill[circleClump[i]]++] = i;
        double[] fx = new double[n], fy = new double[n], fr = new double[n];
        int[] fl = new int[n];
        for (int i = 0; i < n; i++) { Fit f = fits.get(i); fx[i] = f.cx; fy[i] = f.cy; fr[i] = f.r; fl[i] = f.label; }
        int chunks = Math.max(1, Math.min(Math.max(1, THREADS), insideCount / 2000));
        List<Runnable> jobs = new ArrayList<>();
        for (int part = 0; part < chunks; part++) {
            final int from = (int) ((long) insideCount * part / chunks), to = (int) ((long) insideCount * (part + 1) / chunks);
            jobs.add(() -> {
                for (int k = from; k < to; k++) {
                    int p = insideIdx[k];
                    int c = clumps[p];
                    if (starts[c] == starts[c + 1]) continue;
                    if (starts[c] + 1 == starts[c + 1]) { sections[p] = fl[members[starts[c]]]; continue; }
                    int y = p / W, x = p - y * W, closest = -1, winner = -1;
                    double closestGap = Double.POSITIVE_INFINITY;
                    for (int m = starts[c]; m < starts[c + 1]; m++) {
                        int i = members[m];
                        double gap = dist(x - fx[i], y - fy[i]) - fr[i];
                        if (gap < closestGap) { closestGap = gap; closest = i; }
                        if (gap <= 0) {
                            if (winner < 0) winner = i;
                            else { int front = known[winner][i]; if (front == fl[i] || (front == -1 && fy[i] > fy[winner])) winner = i; }
                        }
                    }
                    sections[p] = winner >= 0 ? fl[winner] : fl[closest];
                }
            });
        }
        runAll(jobs);
        return sections;
    }

    static int[] rPixStart = new int[0], rPix = new int[0], rPixX = new int[0], rPixY = new int[0];
    static double[][] rPrevSig = new double[0][];
    static final double[] R_NO_SIG = new double[0];
    static int rClumpN;
    static boolean rFirst;

    // Per-clump pixel lists (ascending insideIdx order) for the refit loop; resets incremental assignment state.
    static void rRefitInit(int[] clumps, int clumpCount) {
        int n = insideCount;
        int[] idx = insideIdx;
        if (rPixStart.length < clumpCount + 1) rPixStart = new int[clumpCount + 1]; else Arrays.fill(rPixStart, 0, clumpCount + 1, 0);
        if (rPix.length < n) { rPix = new int[n]; rPixX = new int[n]; rPixY = new int[n]; }
        for (int k = 0; k < n; k++) rPixStart[clumps[idx[k]] + 1]++;
        for (int c = 0; c < clumpCount; c++) rPixStart[c + 1] += rPixStart[c];
        int[] fill = Arrays.copyOf(rPixStart, clumpCount);
        for (int k = 0; k < n; k++) {
            int p = idx[k], c = clumps[p], at = fill[c]++, y = p / W;
            rPix[at] = p; rPixY[at] = y; rPixX[at] = p - y * W;
        }
        if (assignBuf.length != H * W) assignBuf = new int[H * W];
        if (pieceBuf.length < H * W) { pieceBuf = new int[H * W]; queueBuf = new int[H * W]; }
        rPrevSig = new double[clumpCount][];
        rClumpN = clumpCount;
        rFirst = true;
    }

    // Equivalent to largestPieces(assignToCircles(...)) written into out, restricted to clumps whose member circles changed since
    // the previous call. Returns whether any inside pixel of out differs from ref (out may be ref, in place).
    static boolean rRefitRound(int[] clumps, List<Fit> fits, Map<Long, Integer> fronts, int[] out, int[] ref) {
        final int cn = rClumpN, n = fits.size();
        final int[] sections = assignBuf, piece = pieceBuf;
        int[] circleClump = new int[n];
        for (int i = 0; i < n; i++) { Fit f = fits.get(i); circleClump[i] = clumps[Math.min(Math.max((int) Math.rint(f.my), 0), H - 1) * W + Math.min(Math.max((int) Math.rint(f.mx), 0), W - 1)]; }
        final int[] starts = new int[cn + 1];
        for (int i = 0; i < n; i++) starts[circleClump[i] + 1]++;
        for (int c = 0; c < cn; c++) starts[c + 1] += starts[c];
        final int[] members = new int[n], fill = starts.clone();
        for (int i = 0; i < n; i++) members[fill[circleClump[i]]++] = i;
        final double[] fx = new double[n], fy = new double[n], fr = new double[n];
        final int[] fl = new int[n];
        int maxLabel = 0;
        for (int i = 0; i < n; i++) { Fit f = fits.get(i); fx[i] = f.cx; fy[i] = f.cy; fr[i] = f.r; fl[i] = f.label; if (f.label > maxLabel) maxLabel = f.label; }
        final int labelCap = maxLabel + 1;
        int[] todo = new int[cn];
        int nt = 0;
        long totalPix = 0;
        for (int c = 0; c < cn; c++) {
            int m0 = starts[c], m1 = starts[c + 1], m = m1 - m0;
            double[] old = rPrevSig[c];
            double[] sig = R_NO_SIG;
            boolean diff = rFirst || old == null || old.length != 4 * m;
            if (m > 0) {
                sig = new double[4 * m];
                for (int q = 0; q < m; q++) { int i = members[m0 + q]; sig[4 * q] = fl[i]; sig[4 * q + 1] = fx[i]; sig[4 * q + 2] = fy[i]; sig[4 * q + 3] = fr[i]; }
                if (!diff) for (int q = 0; q < sig.length; q++) if (sig[q] != old[q]) { diff = true; break; }
            }
            if (!diff) continue;
            rPrevSig[c] = sig;
            int pc = rPixStart[c + 1] - rPixStart[c];
            if (pc == 0) continue;
            todo[nt++] = c;
            totalPix += pc;
        }
        rFirst = false;
        if (nt == 0) return false;
        int chunks = (int) Math.max(1, Math.min(Math.max(1, THREADS), totalPix / 2000));
        List<Runnable> jobs = new ArrayList<>();
        final boolean[] changedFlags = new boolean[chunks];
        int from = 0;
        long acc = 0;
        for (int part = 0; part < chunks; part++) {
            int to = from;
            long goal = totalPix * (part + 1) / chunks;
            if (part == chunks - 1) to = nt;
            else while (to < nt && acc < goal) { acc += rPixStart[todo[to] + 1] - rPixStart[todo[to]]; to++; }
            final int jf = from, jt = to, jp = part;
            from = to;
            if (jf == jt) continue;
            jobs.add(() -> {
                int maxPix = 0;
                for (int t = jf; t < jt; t++) maxPix = Math.max(maxPix, rPixStart[todo[t] + 1] - rPixStart[todo[t]]);
                int[] queue = new int[maxPix];
                int[] pieceSize = new int[maxPix + 1], pieceLabel = new int[maxPix + 1];
                int[] bestPiece = new int[labelCap], bestSize = new int[labelCap];
                Arrays.fill(bestPiece, -1);
                boolean changed = false;
                for (int t = jf; t < jt; t++) {
                    int c = todo[t], a = rPixStart[c], b = rPixStart[c + 1], m0 = starts[c], m1 = starts[c + 1];
                    if (m0 == m1 || m0 + 1 == m1) {
                        // zero or one circle: the whole clump (one 8-connected piece) gets its label
                        int v = m0 == m1 ? 0 : Math.max(fl[members[m0]], 0);
                        for (int u = a; u < b; u++) { int i = rPix[u]; if (ref[i] != v) changed = true; out[i] = v; }
                        continue;
                    }
                    {
                        int mm = m1 - m0;
                        int[] kn = new int[mm * mm];
                        for (int ii = 0; ii < mm; ii++) for (int jj = 0; jj < mm; jj++) {
                            int la = fl[members[m0 + ii]], lb = fl[members[m0 + jj]];
                            Integer v = fronts.get(key(Math.min(la, lb), Math.max(la, lb)));
                            kn[ii * mm + jj] = v == null ? -1 : v;
                        }
                        double[] lo2 = new double[mm], hi2 = new double[mm];
                        for (int q = 0; q < mm; q++) { double r = fr[members[m0 + q]]; double rl = r * (1 - 1e-9), rh = r * (1 + 1e-9); lo2[q] = rl * rl; hi2[q] = rh * rh; }
                        for (int u = a; u < b; u++) {
                            int x = rPixX[u], y = rPixY[u], winner = -1;
                            // phase 1: members containing the pixel (sqrt only when the squared distance is within 1e-9 of the radius)
                            for (int q = 0; q < mm; q++) {
                                int i = members[m0 + q];
                                double dx = x - fx[i], dy = y - fy[i], d2 = dx * dx + dy * dy;
                                if (d2 > hi2[q]) continue;
                                if (d2 > lo2[q] && Math.sqrt(d2) - fr[i] > 0) continue;
                                if (winner < 0) winner = q;
                                else { int front = kn[winner * mm + q]; if (front == fl[i] || (front == -1 && fy[i] > fy[members[m0 + winner]])) winner = q; }
                            }
                            if (winner < 0) {
                                double closestGap = Double.POSITIVE_INFINITY;
                                for (int q = 0; q < mm; q++) {
                                    int i = members[m0 + q];
                                    double gap = dist(x - fx[i], y - fy[i]) - fr[i];
                                    if (gap < closestGap) { closestGap = gap; winner = q; }
                                }
                            }
                            sections[rPix[u]] = fl[members[m0 + winner]];
                        }
                    }
                    // largest connected piece per label within this clump
                    for (int u = a; u < b; u++) piece[rPix[u]] = -1;
                    int count = 0;
                    for (int u = a; u < b; u++) {
                        int i0 = rPix[u];
                        int label = sections[i0];
                        if (label <= 0 || piece[i0] >= 0) continue;
                        piece[i0] = count;
                        queue[0] = i0;
                        int head = 0, tail = 1;
                        while (head < tail) {
                            int at = queue[head++], ay = at / W, ax = at - ay * W;
                            if (ay > 0 && ay < H - 1 && ax > 0 && ax < W - 1) {
                                int j = at - W - 1;
                                if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                                j++;
                                if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                                j++;
                                if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                                j = at - 1;
                                if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                                j += 2;
                                if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                                j = at + W - 1;
                                if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                                j++;
                                if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                                j++;
                                if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                                continue;
                            }
                            int y0 = ay > 0 ? ay - 1 : 0, y1 = ay + 2 < H ? ay + 2 : H, x0 = ax > 0 ? ax - 1 : 0, x1 = ax + 2 < W ? ax + 2 : W;
                            for (int yy = y0; yy < y1; yy++) {
                                int row = yy * W;
                                for (int xx = x0; xx < x1; xx++) {
                                    int j = row + xx;
                                    if (sections[j] == label && piece[j] < 0) { piece[j] = count; queue[tail++] = j; }
                                }
                            }
                        }
                        pieceSize[count] = tail;
                        pieceLabel[count] = label;
                        count++;
                    }
                    for (int pc = 0; pc < count; pc++) { int l = pieceLabel[pc]; if (pieceSize[pc] > bestSize[l]) { bestSize[l] = pieceSize[pc]; bestPiece[l] = pc; } }
                    for (int u = a; u < b; u++) {
                        int i = rPix[u], l = sections[i];
                        int v = l > 0 && bestPiece[l] == piece[i] ? l : 0;
                        if (ref[i] != v) changed = true;
                        out[i] = v;
                    }
                    for (int pc = 0; pc < count; pc++) { int l = pieceLabel[pc]; bestSize[l] = 0; bestPiece[l] = -1; }
                }
                changedFlags[jp] = changed;
            });
        }
        runAll(jobs);
        for (boolean ch : changedFlags) if (ch) return true;
        return false;
    }

    static byte[] dRemRem = new byte[0], dRemOut = new byte[0], dRemOpened = new byte[0];
    static int[] dRemLab = new int[0];
    static float[] dRemThick = new float[0];

    static List<Found> remainingBalls(byte[] filled, List<double[]> circles, Expect expected) {
        List<Found> estimated = new ArrayList<>();
        if (dRemRem.length != H * W) { dRemRem = new byte[H * W]; dRemOut = new byte[H * W]; dRemOpened = new byte[H * W]; dRemLab = new int[H * W]; dRemThick = new float[H * W]; }
        byte[] rem = dRemRem;
        for (int i = 0; i < H * W; i++) rem[i] = (byte) (filled[i] != 0 ? 1 : 0);
        Mat remaining = bytesMat(rem, H, W);
        long s = tic();
        for (double[] c : circles) Imgproc.circle(remaining, new Point(Math.rint(c[0]), Math.rint(c[1])), (int) Math.rint(c[2]), new Scalar(0), -1);
        toc(s);
        remaining.get(0, 0, dRemOut);
        pRemaining = dRemOut;
        if (expected == null) return estimated;
        Mat labels = tmp(), stats = tmp(), cents = tmp();
        s = tic();
        int count = Imgproc.connectedComponentsWithStats(remaining, labels, stats, cents, 8, CvType.CV_32S);
        toc(s);
        int[] lab = dRemLab, st = getInts(stats);
        labels.get(0, 0, lab);
        double[] mid = new double[count * 2];
        cents.get(0, 0, mid);
        byte[] opened = dRemOpened;
        Arrays.fill(opened, (byte) 0);
        for (int p = 1; p < count; p++) {
            int left = st[p * 5], top = st[p * 5 + 1], bw = st[p * 5 + 2], bh = st[p * 5 + 3];
            int reach = (int) Math.rint(remainingOpenPctg * Math.max(expected.at(mid[p * 2 + 1]), 1.0));
            if (reach > 0 && Math.min(bw, bh) < 2 * reach + 1 && top - reach >= 0 && left - reach >= 0 && top + bh + reach <= H && left + bw + reach <= W) continue;
            int y0 = Math.max(top - reach, 0), y1 = Math.min(top + bh + reach, H), x0 = Math.max(left - reach, 0), x1 = Math.min(left + bw + reach, W);
            int pw = x1 - x0, ph = y1 - y0;
            byte[] part = new byte[pw * ph];
            for (int y = y0; y < y1; y++) for (int x = x0; x < x1; x++) if (lab[y * W + x] == p) part[(y - y0) * pw + x - x0] = 1;
            if (reach > 0) {
                Mat pm = bytesMat(part, ph, pw), po = tmp();
                s = tic();
                Imgproc.morphologyEx(pm, po, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(2 * reach + 1, 2 * reach + 1)));
                toc(s);
                part = getBytes(po);
            }
            for (int y = y0; y < y1; y++) for (int x = x0; x < x1; x++) opened[y * W + x] |= part[(y - y0) * pw + x - x0];
        }
        Mat om = bytesMat(opened, H, W), thick = tmp();
        labels = tmp(); stats = tmp(); cents = tmp();
        s = tic();
        count = Imgproc.connectedComponentsWithStats(om, labels, stats, cents, 8, CvType.CV_32S);
        Imgproc.distanceTransform(om, thick, Imgproc.DIST_L2, 5);
        toc(s);
        lab = dRemLab; st = getInts(stats);
        labels.get(0, 0, lab);
        mid = new double[count * 2];
        cents.get(0, 0, mid);
        float[] th = dRemThick;
        thick.get(0, 0, th);
        for (int p = 1; p < count; p++) {
            int area = st[p * 5 + 4];
            double e = Math.max(expected.at(mid[p * 2 + 1]), 1.0);
            if (area < remainingMinBallPctg * Math.PI * e * e) continue;
            int left = st[p * 5], top = st[p * 5 + 1], bw = st[p * 5 + 2], bh = st[p * 5 + 3];
            int[] px = new int[area], py = new int[area];
            int n = 0;
            float radius = 0;
            for (int y = top; y < top + bh; y++) for (int x = left; x < left + bw; x++) if (lab[y * W + x] == p) { px[n] = x; py[n] = y; n++; radius = Math.max(radius, th[y * W + x]); }
            if (radius < remainingMinRadiusPx) continue;
            int number = Math.max((int) Math.rint(area / (remainingBallAreaPctg * Math.PI * radius * radius)), 1);
            for (double[] c : spreadCenters(px, py, n, Math.min(number, n))) { Found b = new Found(); b.x = c[0]; b.y = c[1]; b.r = radius; b.estimated = true; estimated.add(b); }
        }
        return estimated;
    }

    static List<double[]> spreadCenters(int[] px, int[] py, int n, int count) {
        double mx = 0, my = 0;
        for (int i = 0; i < n; i++) { mx += px[i]; my += py[i]; }
        mx /= n; my /= n;
        double[] cx = new double[count], cy = new double[count];
        int have = 0;
        if (count > 1) {
            int far = 0;
            double fd = -1;
            for (int i = 0; i < n; i++) { double d = dist(px[i] - mx, py[i] - my); if (d > fd) { fd = d; far = i; } }
            cx[0] = px[far]; cy[0] = py[far]; have = 1;
        } else { cx[0] = mx; cy[0] = my; have = 1; }
        while (have < count) {
            int far = 0;
            double fd = -1;
            for (int i = 0; i < n; i++) {
                double d = Double.MAX_VALUE;
                for (int c = 0; c < have; c++) d = Math.min(d, dist(px[i] - cx[c], py[i] - cy[c]));
                if (d > fd) { fd = d; far = i; }
            }
            cx[have] = px[far]; cy[have] = py[far]; have++;
        }
        double[] sx = new double[count], sy = new double[count], sn = new double[count];
        for (int it = 0; it < 15; it++) {
            Arrays.fill(sx, 0); Arrays.fill(sy, 0); Arrays.fill(sn, 0);
            for (int i = 0; i < n; i++) {
                int best = 0;
                double bd = Double.MAX_VALUE;
                for (int c = 0; c < count; c++) { double d = dist(px[i] - cx[c], py[i] - cy[c]); if (d < bd) { bd = d; best = c; } }
                sx[best] += px[i]; sy[best] += py[i]; sn[best]++;
            }
            boolean close = true;
            double[] nx = new double[count], ny = new double[count];
            for (int c = 0; c < count; c++) {
                nx[c] = sn[c] > 0 ? sx[c] / sn[c] : cx[c];
                ny[c] = sn[c] > 0 ? sy[c] / sn[c] : cy[c];
                if (Math.abs(nx[c] - cx[c]) > 1e-8 + 1e-5 * Math.abs(cx[c]) || Math.abs(ny[c] - cy[c]) > 1e-8 + 1e-5 * Math.abs(cy[c])) close = false;
            }
            if (close) break;
            cx = nx; cy = ny;
        }
        List<double[]> centers = new ArrayList<>();
        for (int c = 0; c < count; c++) centers.add(new double[]{cx[c], cy[c]});
        return centers;
    }

    static double[][] calibration;
    static Mat calMatrix, calDist;
    static int cutoffRow = -1;

    static void loadCalibration() {
        double[] k = calK.clone(), d = calD.clone();
        double cw = calW, ch = calH;
        if ((cw > ch) != (W > H)) {
            k = new double[]{calK[4], calK[3], calK[5], calK[1], calK[0], calK[2], calK[7], calK[6], calK[8]};
            double t = d[2]; d[2] = d[3]; d[3] = t;
            cw = calH; ch = calW;
        }
        for (int j = 0; j < 3; j++) { k[j] *= W / cw; k[3 + j] *= H / ch; }
        k[6] = 0; k[7] = 0; k[8] = 1;
        calMatrix = new Mat(3, 3, CvType.CV_64F);
        calMatrix.put(0, 0, k);
        calDist = new Mat(1, d.length, CvType.CV_64F);
        calDist.put(0, 0, d);
    }

    static double[] transform(double x, double y, double z) {
        // camera frame x right, y forward, z up; result in cm, robot frame (x right, y forward, z up from the floor)
        double p = Math.toRadians(camPitchDownDeg);
        double yp = y * Math.cos(p) + z * Math.sin(p), zp = z * Math.cos(p) - y * Math.sin(p);
        double w = Math.toRadians(camYawLeftDeg);
        return new double[]{x * Math.cos(w) - yp * Math.sin(w) - camLeftIn * CM, x * Math.sin(w) + yp * Math.cos(w) + camForwardIn * CM, zp + camHeightIn * CM};
    }

    static double[] floorPoint(double px, double py, double heightCm) {
        MatOfPoint2f src = new MatOfPoint2f(new Point(px, py)), dst = new MatOfPoint2f();
        long s = tic();
        Calib3d.undistortPoints(src, dst, calMatrix, calDist);
        toc(s);
        Point u = dst.toArray()[0];
        src.release();
        dst.release();
        double norm = Math.sqrt(u.x * u.x + u.y * u.y + 1);
        double xd = u.x / norm, yd = 1 / norm, zd = -u.y / norm;
        double down = transform(xd, yd, zd)[2] - camHeightIn * CM;
        if (down >= -1e-6) return null;
        double distance = (heightCm - camHeightIn * CM) / down;
        double[] r = transform(xd * distance, yd * distance, zd * distance);
        return new double[]{r[0], r[1], yd * distance, distance};
    }

    static int fieldCutoffRow() {
        if (cutoffRow >= 0) return cutoffRow;
        double[] origin = floorPoint((W - 1) / 2.0, H - 1, 0);
        double oy = origin != null ? origin[1] : 0;
        int low = 0, high = H;
        while (low < high) {
            int middle = (low + high) / 2;
            double[] p = floorPoint((W - 1) / 2.0, middle, ballRadiusCm);
            if (p == null || p[1] - oy > fieldMaxDistanceCm) low = middle + 1; else high = middle;
        }
        cutoffRow = low;
        return low;
    }

    static List<Found> locateBalls(List<Fit> fits, List<Found> estimated) {
        int cutoff = fieldCutoffRow();
        List<Found> balls = new ArrayList<>();
        for (double[] c : accepted) { Found b = new Found(); b.x = c[0]; b.y = c[1]; b.r = c[2]; balls.add(b); }
        for (Fit f : fits) { Found b = new Found(); b.x = f.cx; b.y = f.cy; b.r = f.r; balls.add(b); }
        balls.addAll(estimated);
        double[] origin = floorPoint((W - 1) / 2.0, H - 1, 0);
        double focal = (calMatrix.get(0, 0)[0] + calMatrix.get(1, 1)[0]) / 2;
        for (Found b : balls) {
            double[] p = floorPoint(b.x, b.y, ballRadiusCm);
            b.raw = p;
            if (p != null) {
                double rx = p[0], ry = p[1];
                if (origin != null) { rx -= origin[0]; ry -= origin[1]; }
                b.position = new double[]{rx, ry};
                if (p[2] > 1e-6) {
                    double e = focal * ballRadiusCm / 2 * (1 / p[2] + p[3] / (p[2] * p[2]));
                    b.expected = e;
                    b.kept = Math.abs(b.r * b.r - e * e) <= sizeFilterPctg * e * e;
                }
            }
            b.kept = b.kept && ballMinRadiusPx <= b.r && b.r <= ballMaxRadiusPx;
            if (b.y < cutoff) b.kept = false;
        }
        return balls;
    }

    static final List<Mat> temps = new ArrayList<>();
    static final List<MatOfPoint> contourPool = new ArrayList<>();

    static double dist(double dx, double dy) { return Math.sqrt(dx * dx + dy * dy); }

    static Mat track(Mat m) { temps.add(m); return m; }
    static Mat tmp() { return track(new Mat()); }
    static Mat tmp(int rows, int cols, int type) { return track(new Mat(rows, cols, type)); }
    static Mat tmpZeros(int rows, int cols, int type) { return track(Mat.zeros(rows, cols, type)); }
    static Mat tmpClone(Mat m) { return track(m.clone()); }
    static MatOfPoint tmpPoints(Point[] points) { MatOfPoint m = new MatOfPoint(points); temps.add(m); return m; }

    static void findContoursTracked(Mat image, List<MatOfPoint> contours, Mat hierarchy, int mode, int method) {
        Imgproc.findContours(image, contours, hierarchy, mode, method);
        contourPool.addAll(contours);
    }

    static void releaseTemps() {
        for (Mat m : temps) m.release();
        temps.clear();
        for (MatOfPoint c : contourPool) c.release();
        contourPool.clear();
    }

    static java.lang.reflect.Method inpaintMethod;
    static int inpaintFlag;
    static boolean inpaintLooked;

    static synchronized void findInpaint() {
        if (inpaintLooked) return;
        try {
            Class<?> photo = Class.forName("org.opencv.photo.Photo");
            inpaintFlag = photo.getField("INPAINT_TELEA").getInt(null);
            inpaintMethod = photo.getMethod("inpaint", Mat.class, Mat.class, Mat.class, double.class, int.class);
        } catch (Throwable error) {
            inpaintMethod = null;
        }
        inpaintLooked = true;
    }

    static void inpaintTelea(Mat src, Mat mask, Mat dst) {
        findInpaint();
        java.lang.reflect.Method method = inpaintMethod;
        if (method != null) {
            try {
                method.invoke(null, src, mask, dst, 3.0, inpaintFlag);
                return;
            } catch (Throwable error) {
                inpaintMethod = null;
            }
        }
        int rows = src.rows(), cols = src.cols();
        byte[] colors = getBytes(src);
        byte[] bad = getBytes(mask);
        byte[] out = colors.clone();
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                if (bad[y * cols + x] == 0) continue;
                int[] sum = new int[3];
                int count = 0;
                for (int dy = -3; dy <= 3; dy++) {
                    int ny = y + dy;
                    if (ny < 0 || ny >= rows) continue;
                    for (int dx = -3; dx <= 3; dx++) {
                        int nx = x + dx;
                        if (nx < 0 || nx >= cols || bad[ny * cols + nx] != 0) continue;
                        for (int c = 0; c < 3; c++) sum[c] += colors[(ny * cols + nx) * 3 + c] & 255;
                        count++;
                    }
                }
                if (count > 0) for (int c = 0; c < 3; c++) out[(y * cols + x) * 3 + c] = (byte) ((sum[c] + count / 2) / count);
            }
        }
        dst.create(rows, cols, src.type());
        dst.put(0, 0, out);
    }

    static LinkedHashMap<String, Long> stageNanos = new LinkedHashMap<>();
    static long stageStart;
    static void stage(String name) { long now = System.nanoTime(); stageNanos.merge(name, now - stageStart, Long::sum); stageStart = now; }

    static Mat pImage, pRaw;
    static byte[] pMask, pSmoothed, pFilled, pRemaining;
    static float[] pLevel;
    static boolean[] pValleys;
    static List<Conn> pConns = new ArrayList<>();
    static List<double[]> pArcs = new ArrayList<>();
    static double[][] pGrown = new double[0][];
    static List<double[][]> pPaths = new ArrayList<>();
    static int[] pClumps = new int[0], pSections = new int[0], pRefitSections = new int[0];
    static List<Fit> pFirstFits = new ArrayList<>(), pRefitFits = new ArrayList<>();
    static List<Found> pEstimated = new ArrayList<>(), pBalls = new ArrayList<>();

    static List<Found> run(Mat raw) {
        H = raw.rows(); W = raw.cols();
        stageStart = System.nanoTime();
        pRaw = raw;
        Mat image = whiteBalance(raw);
        pImage = image;
        byte[] mask = yellowMask(image);
        byte[] filled = fillHoles(mask);
        pMask = mask;
        pFilled = filled;
        stage("mask");
        aBgrBuf = aBuf(aBgrBuf, H * W * 3);
        aGetBytesInto(image, aBgrBuf);
        byte[] bgr = aBgrBuf;
        byte[] brightness = lastL;
        byte[][] blemishOut = new byte[1][];
        byte[] smoothed = smoothInside(bgr, brightness, mask, filled, blemishOut);
        pSmoothed = smoothed;
        stage("smoothing");
        singleBalls(filled);
        aRemBuf = aBuf(aRemBuf, H * W);
        byte[] remaining = aRemBuf;
        long s = tic();
        if (acceptedContours.isEmpty()) System.arraycopy(filled, 0, remaining, 0, H * W);
        else {
            aRemMat = aMat(aRemMat, H, W, CvType.CV_8UC1);
            aRemMat.put(0, 0, filled);
            Imgproc.drawContours(aRemMat, acceptedContours, -1, new Scalar(0), -1);
            toc(s);
            aGetBytesInto(aRemMat, remaining);
        }
        byte[] smoothL = aSmoothL(smoothed);
        float[] level = brightnessRanks(smoothL, remaining);
        pLevel = level;
        stage("single balls + height map");
        boolean[] inside = new boolean[H * W];
        byte[] insideBytes = new byte[H * W];
        for (int i = 0; i < H * W; i++) if (level[i] >= 0) { inside[i] = true; insideBytes[i] = 1; }
        indexInside(inside);
        List<double[]> refs = new ArrayList<>();
        for (double[] c : accepted) refs.add(new double[]{c[1], c[2]});
        Expect singleExpected = radiusByRow(refs);
        boolean[] valleys = valleyLines(level, singleExpected);
        pValleys = valleys;
        stage("valley lines");
        seedPoints(inside, insideBytes, level, valleys);
        stage("seeds");
        int[] cnt = new int[1];
        int[] clumps = connected(insideBytes, 8, cnt, 2);
        pClumps = clumps;
        int clumpCount = cnt[0];
        List<Conn> first = spotConnections(clumps, singleExpected, valleys, false);
        stage("first connections");
        int[] firstSections = sectionClumps(clumps, first);
        stage("first sections");
        List<Fit> firstFits = clumpCircles(firstSections, inside, filled);
        stage("first circles");
        List<double[]> refs2 = new ArrayList<>(refs);
        for (Fit f : firstFits) if (f.score >= fitMinPctg && f.outer >= edgeMinPoints && !f.fixed) refs2.add(new double[]{f.cy, f.r});
        Expect expected = radiusByRow(refs2);
        List<Conn> conns = spotConnections(clumps, expected, valleys, true);
        stage("connections + pair arcs");
        ballArcs(conns, firstSections, inside, expected);
        pConns = conns;
        stage("one arc per ball");
        List<double[]> arcs = shiftArcs(conns, smoothL, inside);
        pArcs = arcs;
        stage("shift");
        double[][] grown = extendArcs(arcs, inside);
        pGrown = grown;
        stage("extend");
        List<double[][]> paths = snapArcs(arcs, grown, inside, insideBytes);
        pPaths = paths;
        stage("snap");
        int[] sections = sectionByArcs(inside, paths, sectionClumps(clumps, conns));
        pSections = sections;
        stage("split");
        List<Fit> fits = clumpCircles(sections, inside, filled);
        pFirstFits = fits;
        stage("fit");
        Map<Long, Integer> fronts = frontPairs(conns);
        stage("fit");
        rRefitInit(clumps, clumpCount);
        for (int round = 0; round < refitRounds; round++) {
            if (round == 0) {
                int[] target = refitBufs[0];
                if (target.length != H * W) { target = new int[H * W]; refitBufs[0] = target; } else Arrays.fill(target, 0);
                if (!rRefitRound(clumps, fits, fronts, target, sections)) break;
                sections = target;
            } else {
                if (!rRefitRound(clumps, fits, fronts, sections, sections)) break;
            }
            fits = rClumpCirclesInc(sections, inside, filled);
        }
        pRefitSections = sections;
        pRefitFits = fits;
        stage("refit");
        List<double[]> circles = new ArrayList<>(accepted);
        for (Fit f : fits) circles.add(new double[]{f.cx, f.cy, f.r});
        List<Found> estimated = remainingBalls(filled, circles, seedExpected);
        pEstimated = estimated;
        stage("remaining mass");
        List<Found> out = locateBalls(fits, estimated);
        pBalls = out;
        stage("locate");
        return out;
    }

    static int[] tableSignature;
    static double[] geometrySignature;

    static void prepare(int width, int height) {
        W = width;
        H = height;
        int[] signature = {minHue, maxHue, minChroma};
        if (colorTable == null || !Arrays.equals(signature, tableSignature)) {
            colorTable = makeColorTable();
            tableSignature = signature;
        }
        double[] geometry = {width, height, camPitchDownDeg, camYawLeftDeg, camLeftIn, camForwardIn, camHeightIn, ballRadiusCm, fieldMaxDistanceCm};
        if (!Arrays.equals(geometry, geometrySignature)) {
            geometrySignature = geometry;
            if (calMatrix != null) { calMatrix.release(); calDist.release(); }
            loadCalibration();
            cutoffRow = -1;
        }
    }

    public static final String[] MODE_NAMES = {
            "0 raw", "1 white balance", "2 mask", "3 smoothed", "4 single balls", "5 height map",
            "6 b&w height map", "7 valley lines", "8 seed votes", "9 seeds", "10 spot connections",
            "11 arc shift", "12 arc extend", "13 merged outline", "14 sections", "15 circles",
            "16 refit", "17 size filter", "18 remaining mask", "19 center dots", "20 centers",
            "21 undistorted centers"
    };

    private static final Scalar RED = new Scalar(0, 0, 255);
    private static final Scalar GREEN = new Scalar(0, 255, 0);
    private static final Scalar BLUE = new Scalar(255, 0, 0);
    private static final Scalar CYAN = new Scalar(255, 255, 0);
    private static final Scalar YELLOW = new Scalar(0, 255, 255);
    private static final Scalar ORANGE = new Scalar(0, 165, 255);
    private static final int FONT = Imgproc.FONT_HERSHEY_SIMPLEX;

    static String panelLabel = "";
    static byte[] hueColors;

    static void ensureHueColors() {
        if (hueColors != null) return;
        Mat hsv = new Mat(1, 180, CvType.CV_8UC3);
        byte[] values = new byte[180 * 3];
        for (int h = 0; h < 180; h++) { values[h * 3] = (byte) h; values[h * 3 + 1] = (byte) 255; values[h * 3 + 2] = (byte) 255; }
        hsv.put(0, 0, values);
        Mat bgrColors = new Mat();
        Imgproc.cvtColor(hsv, bgrColors, Imgproc.COLOR_HSV2BGR);
        hueColors = getBytes(bgrColors);
        hsv.release();
        bgrColors.release();
    }

    static void paint(byte[] data, int pixel, int hue) {
        data[pixel * 3] = hueColors[hue * 3];
        data[pixel * 3 + 1] = hueColors[hue * 3 + 1];
        data[pixel * 3 + 2] = hueColors[hue * 3 + 2];
    }

    static void paint(byte[] data, int pixel, int b, int g, int r) {
        data[pixel * 3] = (byte) b;
        data[pixel * 3 + 1] = (byte) g;
        data[pixel * 3 + 2] = (byte) r;
    }

    static Mat heightMap(boolean colored) {
        ensureHueColors();
        Mat view = tmpClone(pImage);
        byte[] d = getBytes(view);
        for (int i = 0; i < H * W; i++) {
            if (pLevel[i] < 0) continue;
            float f = Math.min(Math.max(pLevel[i], 0f), 1f);
            if (colored) paint(d, i, (int) ((1 - f) * 150));
            else { int g = (int) (f * 255); paint(d, i, g, g, g); }
        }
        view.put(0, 0, d);
        return view;
    }

    static void arc(Mat image, double cx, double cy, double r, double start, double end, Scalar color) {
        Imgproc.ellipse(image, new Point(Math.rint(cx * 16), Math.rint(cy * 16)), new Size(Math.rint(r * 16), Math.rint(r * 16)), 0, start, end, color, 1, Imgproc.LINE_8, 4);
    }

    static Point rounded(double x, double y) { return new Point(Math.rint(x), Math.rint(y)); }

    static Mat arcWalls() {
        Mat walls = tmpZeros(H, W, CvType.CV_8UC1);
        for (double[][] p : pPaths) {
            int n = p[0].length;
            if (n >= 2) {
                Point[] pts = new Point[n];
                for (int j = 0; j < n; j++) pts[j] = new Point(Math.rint(p[0][j] * 16), Math.rint(p[1][j] * 16));
                Imgproc.polylines(walls, Collections.singletonList(tmpPoints(pts)), false, new Scalar(255), 1, Imgproc.LINE_8, 4);
            } else if (n == 1) walls.put((int) Math.rint(p[1][0]), (int) Math.rint(p[0][0]), 255);
        }
        return walls;
    }

    static Mat drawSections(int[] sections) {
        ensureHueColors();
        Mat view = tmpClone(pImage);
        byte[] d = getBytes(view);
        for (int i = 0; i < H * W; i++) {
            if (sections[i] <= 0) continue;
            int hue = (sections[i] * 47) % 180;
            for (int c = 0; c < 3; c++) d[i * 3 + c] = (byte) (((d[i * 3 + c] & 255) / 2) + ((hueColors[hue * 3 + c] & 255) / 2));
        }
        view.put(0, 0, d);
        for (int i = 0; i < spotX.length; i++) Imgproc.circle(view, new Point(spotX[i], spotY[i]), 1, RED, -1);
        return view;
    }

    static void drawFits(Mat view, List<Fit> fits) {
        for (double[] c : accepted) {
            Point center = rounded(c[0], c[1]);
            Imgproc.circle(view, center, (int) Math.rint(c[2]), CYAN, 1);
            Imgproc.circle(view, center, 1, CYAN, -1);
        }
        for (Fit f : fits) {
            Scalar color = f.fixed ? YELLOW : GREEN;
            Point center = rounded(f.cx, f.cy);
            Imgproc.circle(view, center, (int) Math.rint(f.r), color, 1);
            Imgproc.circle(view, center, 1, color, -1);
        }
    }

    static int keptCount() {
        int n = 0;
        for (Found b : pBalls) if (b.kept) n++;
        return n;
    }

    static Mat buildPanel(int mode) {
        Mat view;
        switch (mode) {
            case 0:
                panelLabel = "raw";
                return tmpClone(pRaw);
            case 1:
                panelLabel = "white balance";
                return tmpClone(pImage);
            case 2: {
                panelLabel = "mask";
                view = tmp();
                Imgproc.cvtColor(bytesMat(pMask, H, W), view, Imgproc.COLOR_GRAY2BGR);
                return view;
            }
            case 3:
                panelLabel = "smoothed";
                view = tmp(H, W, CvType.CV_8UC3);
                view.put(0, 0, pSmoothed);
                return view;
            case 4: {
                view = tmpClone(pImage);
                for (int i = 0; i < rejectedContours.size(); i++) {
                    MatOfPoint contour = rejectedContours.get(i);
                    Imgproc.drawContours(view, Collections.singletonList(contour), -1, RED, 1);
                    Rect box = Imgproc.boundingRect(contour);
                    Imgproc.putText(view, Math.round(rejectedPct.get(i) * 100) + "%", new Point(box.x, Math.max(box.y - 4, 12)), FONT, 0.5, RED, 2);
                }
                for (int i = 0; i < accepted.size(); i++) {
                    double[] c = accepted.get(i);
                    Imgproc.circle(view, rounded(c[0], c[1]), (int) Math.rint(c[2]), GREEN, 1);
                    Imgproc.circle(view, rounded(c[0], c[1]), 3, GREEN, -1);
                    Imgproc.putText(view, Math.round(acceptedPct.get(i) * 100) + "%", new Point((int) (c[0] - c[2]), Math.max((int) (c[1] - c[2]) - 4, 12)), FONT, 0.5, GREEN, 2);
                }
                panelLabel = "single balls: " + accepted.size();
                return view;
            }
            case 5:
                panelLabel = "height map";
                return heightMap(true);
            case 6:
                panelLabel = "b&w height map";
                return heightMap(false);
            case 7: {
                view = heightMap(false);
                byte[] d = getBytes(view);
                for (int i = 0; i < H * W; i++) if (pValleys[i]) paint(d, i, 0, 0, 255);
                view.put(0, 0, d);
                List<MatOfPoint> outlines = new ArrayList<>();
                findContoursTracked(bytesMat(pFilled, H, W), outlines, tmp(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
                Imgproc.drawContours(view, outlines, -1, ORANGE, 1);
                panelLabel = "valley lines";
                return view;
            }
            case 8: {
                ensureHueColors();
                view = tmpClone(pImage);
                float most = 0;
                for (int i = 0; i < H * W; i++) if (pLevel[i] >= 0) most = Math.max(most, lastCoverage[i]);
                if (most > 0) {
                    byte[] d = getBytes(view);
                    for (int i = 0; i < H * W; i++) {
                        if (pLevel[i] < 0) continue;
                        float strength = Math.min(Math.max(lastCoverage[i] / Math.max(most, 1e-6f), 0f), 1f);
                        paint(d, i, (int) ((1 - strength) * 150));
                    }
                    view.put(0, 0, d);
                }
                panelLabel = "seed votes";
                return view;
            }
            case 9:
                view = tmpClone(pImage);
                for (int i = 0; i < spotX.length; i++) Imgproc.circle(view, new Point(spotX[i], spotY[i]), 1, RED, -1);
                panelLabel = "seeds: " + spotX.length;
                return view;
            case 10: {
                view = heightMap(false);
                byte[] d = getBytes(view);
                for (int i = 0; i < H * W; i++) if (pValleys[i]) paint(d, i, 0, 0, 160);
                view.put(0, 0, d);
                int guesses = 0, arcCount = 0;
                for (Conn c : pConns) {
                    Imgproc.line(view, rounded(c.sx, c.sy), rounded(c.ex, c.ey), BLUE, 1);
                    if (c.guessed) guesses++;
                    if (c.hasArc) arcCount++;
                }
                Mat arcs = tmpZeros(H, W, CvType.CV_8UC1);
                boolean[] straights = new boolean[H * W];
                for (Conn c : pConns) {
                    if (c.hasArc) {
                        if (c.hasSpan) arc(arcs, c.acx, c.acy, c.ar, c.span0, c.span1, new Scalar(255));
                    } else if (c.guessed) {
                        double dx = c.ex - c.sx, dy = c.ey - c.sy;
                        double length = Math.max(Math.hypot(dx, dy), 1e-6);
                        double nx = -dy / length, ny = dx / length;
                        Mat line = tmpZeros(H, W, CvType.CV_8UC1);
                        Imgproc.line(line, rounded(c.pointX - nx * 2000, c.pointY - ny * 2000), rounded(c.pointX + nx * 2000, c.pointY + ny * 2000), new Scalar(255), 1);
                        byte[] lineData = getBytes(line);
                        int clump = pClumps[(int) c.sy * W + (int) c.sx];
                        for (int i = 0; i < H * W; i++) if (lineData[i] != 0 && pClumps[i] == clump) straights[i] = true;
                    }
                }
                byte[] arcData = getBytes(arcs);
                d = getBytes(view);
                for (int i = 0; i < H * W; i++) if (straights[i]) paint(d, i, 0, 200, 255);
                for (int i = 0; i < H * W; i++) if (arcData[i] != 0 && pLevel[i] >= 0) paint(d, i, 0, 255, 0);
                view.put(0, 0, d);
                for (Conn c : pConns) {
                    Imgproc.circle(view, rounded(c.sx, c.sy), 1, YELLOW, -1);
                    Imgproc.circle(view, rounded(c.ex, c.ey), 1, YELLOW, -1);
                }
                panelLabel = "spot connections: " + pConns.size() + " guesses: " + guesses + " arcs: " + arcCount;
                return view;
            }
            case 11: {
                view = tmpClone(pImage);
                Set<String> seen = new HashSet<>();
                for (Conn c : pConns) {
                    if (!c.hasArc || !c.hasSpan) continue;
                    if (seen.add(Math.rint(c.acx * 1000) + "," + Math.rint(c.acy * 1000) + "," + Math.rint(c.ar * 1000))) arc(view, c.acx, c.acy, c.ar, c.span0, c.span1, new Scalar(0, 110, 0));
                }
                for (double[] a : pArcs) arc(view, a[0], a[1], a[2], a[3], a[4], GREEN);
                panelLabel = "arc shift: " + pArcs.size() + " arcs";
                return view;
            }
            case 12: {
                view = tmpClone(pImage);
                for (int i = 0; i < pArcs.size(); i++) {
                    double[] a = pArcs.get(i);
                    double[] g = pGrown[i];
                    arc(view, a[0], a[1], a[2], g[0], g[1], CYAN);
                    if (Math.max(a[3], g[0]) < Math.min(a[4], g[1])) arc(view, a[0], a[1], a[2], Math.max(a[3], g[0]), Math.min(a[4], g[1]), GREEN);
                }
                panelLabel = "arc extend: " + pArcs.size() + " arcs";
                return view;
            }
            case 13: {
                view = tmpClone(pImage);
                Mat outline = arcWalls();
                List<MatOfPoint> edges = new ArrayList<>();
                Mat insideMat = tmp(H, W, CvType.CV_8UC1);
                byte[] insideData = new byte[H * W];
                for (int i = 0; i < H * W; i++) insideData[i] = (byte) (pLevel[i] >= 0 ? 1 : 0);
                insideMat.put(0, 0, insideData);
                findContoursTracked(insideMat, edges, tmp(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
                Imgproc.drawContours(outline, edges, -1, new Scalar(255), 1);
                byte[] outlineData = getBytes(outline);
                byte[] d = getBytes(view);
                for (int i = 0; i < H * W; i++) if (outlineData[i] != 0 && pLevel[i] >= 0) paint(d, i, 0, 255, 0);
                view.put(0, 0, d);
                panelLabel = "merged outline";
                return view;
            }
            case 14: {
                view = drawSections(pSections);
                Set<Integer> labels = new HashSet<>();
                for (int v : pSections) if (v > 0) labels.add(v);
                panelLabel = "sections: " + labels.size();
                return view;
            }
            case 15:
                view = tmpClone(pImage);
                drawFits(view, pFirstFits);
                panelLabel = "circles: " + (pFirstFits.size() + accepted.size());
                return view;
            case 16:
                view = drawSections(pRefitSections);
                drawFits(view, pRefitFits);
                panelLabel = "refit: " + (pRefitFits.size() + accepted.size());
                return view;
            case 17: {
                view = tmpClone(pImage);
                for (Found b : pBalls) {
                    Point center = rounded(b.x, b.y);
                    if (b.expected > 0) Imgproc.circle(view, center, Math.max((int) Math.rint(b.expected), 1), CYAN, 1);
                    Imgproc.circle(view, center, Math.max((int) Math.rint(b.r), 1), b.kept ? GREEN : RED, 1);
                }
                panelLabel = "size filter: kept " + keptCount() + " of " + pBalls.size();
                return view;
            }
            case 18: {
                byte[] scaled = new byte[H * W];
                for (int i = 0; i < H * W; i++) scaled[i] = (byte) (pRemaining[i] != 0 ? 255 : 0);
                view = tmp();
                Imgproc.cvtColor(bytesMat(scaled, H, W), view, Imgproc.COLOR_GRAY2BGR);
                for (Found b : pEstimated) {
                    Imgproc.circle(view, rounded(b.x, b.y), (int) Math.rint(b.r), ORANGE, 1);
                    Imgproc.circle(view, rounded(b.x, b.y), 2, ORANGE, -1);
                }
                panelLabel = "remaining mask: " + pEstimated.size() + " estimated";
                return view;
            }
            case 19:
                view = tmpClone(pImage);
                for (Found b : pBalls) if (b.kept) Imgproc.circle(view, rounded(b.x, b.y), 2, b.estimated ? ORANGE : RED, -1);
                panelLabel = "center dots: " + keptCount();
                return view;
            case 20: {
                view = tmpClone(pImage);
                int number = 1;
                for (Found b : pBalls) {
                    if (!b.kept) continue;
                    Point center = rounded(b.x, b.y);
                    Imgproc.circle(view, center, 2, RED, -1);
                    Imgproc.putText(view, String.valueOf(number), new Point(center.x + 4, center.y - 4), FONT, 0.4, RED, 1);
                    number++;
                }
                panelLabel = "centers: " + keptCount();
                return view;
            }
            default: {
                view = tmp();
                undistortView(pImage, view);
                List<Point> kept = new ArrayList<>();
                for (Found b : pBalls) if (b.kept) kept.add(new Point(b.x, b.y));
                if (!kept.isEmpty()) {
                    MatOfPoint2f source = new MatOfPoint2f(kept.toArray(new Point[0]));
                    MatOfPoint2f straight = new MatOfPoint2f();
                    Calib3d.undistortPoints(source, straight, calMatrix, calDist, tmp(), calMatrix);
                    for (Point p : straight.toArray()) Imgproc.circle(view, rounded(p.x, p.y), 2, RED, -1);
                    source.release();
                    straight.release();
                }
                panelLabel = "undistorted centers: " + keptCount();
                return view;
            }
        }
    }

    static List<Mat[]> stripeMaps = new ArrayList<>();
    static int[] stripeRows = new int[0];
    static double[] stripeSignature;

    static void undistortView(Mat src, Mat dst) {
        int rows = src.rows(), cols = src.cols();
        double[] signature = {rows, cols, calMatrix.get(0, 0)[0], calMatrix.get(1, 1)[0], calMatrix.get(0, 2)[0], calMatrix.get(1, 2)[0], calDist.get(0, 0)[0], calDist.get(0, 1)[0], calDist.get(0, 2)[0], calDist.get(0, 3)[0], calDist.get(0, 4)[0]};
        if (!Arrays.equals(signature, stripeSignature)) {
            for (Mat[] maps : stripeMaps) { maps[0].release(); maps[1].release(); }
            stripeMaps.clear();
            int stripe = Math.min(Math.max(1, (1 << 12) / Math.max(cols, 1)), rows);
            int count = (rows + stripe - 1) / stripe;
            stripeRows = new int[count + 1];
            Mat shifted = calMatrix.clone();
            double v0 = calMatrix.get(1, 2)[0];
            Mat identity = Mat.eye(3, 3, CvType.CV_64F);
            for (int k = 0, y = 0; y < rows; y += stripe, k++) {
                int height = Math.min(stripe, rows - y);
                shifted.put(1, 2, v0 - y);
                Mat map1 = new Mat(), map2 = new Mat();
                Calib3d.initUndistortRectifyMap(calMatrix, calDist, identity, shifted, new Size(cols, height), CvType.CV_16SC2, map1, map2);
                stripeMaps.add(new Mat[]{map1, map2});
                stripeRows[k] = y;
            }
            stripeRows[count] = rows;
            shifted.release();
            identity.release();
            stripeSignature = signature;
        }
        dst.create(rows, cols, src.type());
        for (int k = 0; k < stripeMaps.size(); k++) {
            Mat part = dst.rowRange(stripeRows[k], stripeRows[k + 1]);
            Imgproc.remap(src, part, stripeMaps.get(k)[0], stripeMaps.get(k)[1], Imgproc.INTER_LINEAR, Core.BORDER_CONSTANT);
            part.release();
        }
    }

    public static class Ball {
        public final double x;
        public final double y;
        public final double radius;
        public final double rangeCm;
        public final double xRobot;
        public final double yRobot;
        public final double zRobot;
        public final double floorXCm;
        public final double floorYCm;
        public final boolean hasRange;
        public final boolean estimated;

        Ball(double x, double y, double radius, Found found) {
            this.x = x;
            this.y = y;
            this.radius = radius;
            this.estimated = found.estimated;
            this.hasRange = found.raw != null && found.position != null;
            if (hasRange) {
                this.rangeCm = found.raw[3];
                this.xRobot = found.raw[0];
                this.yRobot = found.raw[1];
                this.zRobot = ballRadiusCm;
                this.floorXCm = found.position[0];
                this.floorYCm = found.position[1];
            } else {
                this.rangeCm = this.xRobot = this.yRobot = this.zRobot = this.floorXCm = this.floorYCm = 0;
            }
        }
    }

    public volatile double debugProcessMs = -1;
    public volatile int debugSingles = 0;
    public volatile int debugSeeds = 0;
    public volatile int debugConnections = 0;
    public volatile int debugCircles = 0;
    public volatile int debugEstimated = 0;
    public volatile int debugKept = 0;
    public static volatile double debugMaskPct = 0; // % of the image that passed the colour test, last frame
    public static volatile String debugChroma = ""; // % of image with yellow hue and OK brightness, per chroma floor
    public static volatile String debugCenter = ""; // colour at the image centre, last frame, vs the thresholds
    public volatile String debugStages = "";
    public volatile String debugError = "";

    /** One processed frame's balls; id goes up by one per frame, so a caller can tell a new frame from a re-read. */
    public static final class Frame {
        public final long id;
        public final List<Ball> balls;
        Frame(long id, List<Ball> balls) { this.id = id; this.balls = balls; }
    }

    private volatile Frame latest = new Frame(0, Collections.<Ball>emptyList());
    private volatile Frame fresh; // latest, but null from enable() until a frame started after it is done
    private volatile int epoch; // bumped by enable(); a frame started before that never becomes fresh
    private final VisionPortal portal;
    private boolean cameraLocked;
    private volatile boolean lastTransposed = false;
    private volatile int viewMode = 0;
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    // group F: Mats and view bitmaps reused across frames (frames are processed sequentially)
    private final Mat fTransposed = new Mat(), fWorking = new Mat(), fBgr = new Mat();
    private final Mat fView = new Mat(), fRgbPanel = new Mat();
    private Bitmap fBitmapA, fBitmapB;
    private boolean fUseB;
    private int fFrameCount;

    /** Opens the webcam (webcamName, frameWidth x frameHeight) with this as its only processor, enabled. */
    public Detect(HardwareMap hardwareMap) {
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .setCameraResolution(new android.util.Size(frameWidth, frameHeight))
                .enableLiveView(true)
                .addProcessor(this)
                .build();
    }

    /** Start processing frames. capture() returns null until the first frame taken after this. */
    public void enable() {
        epoch++;
        fresh = null;
        portal.setProcessorEnabled(this, true);
    }

    /** Stop processing frames (frees the CPU, e.g. while driving). The camera keeps streaming. */
    public void disable() {
        portal.setProcessorEnabled(this, false);
    }

    /** The newest processed frame since enable(), or null if there is none yet. Call every loop: also locks the camera. */
    public Frame capture() {
        if (!cameraLocked && portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            lockCamera();
            cameraLocked = true;
        }
        return fresh;
    }

    public void close() {
        portal.close();
    }

    // camera controls only work while streaming; values <= 0 stay on auto
    private void lockCamera() {
        if (whiteBalanceK > 0) {
            WhiteBalanceControl wb = portal.getCameraControl(WhiteBalanceControl.class);
            wb.setMode(WhiteBalanceControl.Mode.MANUAL);
            wb.setWhiteBalanceTemperature(whiteBalanceK);
        }
        if (exposureMs > 0) {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure(exposureMs, TimeUnit.MILLISECONDS);
            if (gain > 0) portal.getCameraControl(GainControl.class).setGain(gain);
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        if (calibration == null || calibration.isDegenerate()) {
            calFromSdk = false; // unknown webcam/resolution: keeps the fallback intrinsics, ground positions will be off
            return;
        }
        float[] d = calibration.distortionCoefficients; // OpenCV order k1 k2 p1 p2 k3 ...
        calK = new double[]{calibration.focalLengthX, 0, calibration.principalPointX, 0, calibration.focalLengthY, calibration.principalPointY, 0, 0, 1};
        calD = new double[]{d[0], d[1], d[2], d[3], d[4]};
        calW = width;
        calH = height;
        calFromSdk = true;
        geometrySignature = null; // reload the calibration on the next frame
    }

    public List<Ball> getBalls() {
        return latest.balls;
    }

    public void setViewMode(int mode) {
        viewMode = Math.max(0, Math.min(MODE_NAMES.length - 1, mode));
    }

    public static String getModeName(int mode) {
        return MODE_NAMES[Math.max(0, Math.min(MODE_NAMES.length - 1, mode))];
    }

    public static int getModeCount() {
        return MODE_NAMES.length;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(consumer -> consumer.accept(lastFrame.get()));
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        long startTime = System.nanoTime();
        int frameEpoch = epoch;
        int mode = viewMode;
        boolean transposed = PORTRAIT;
        Mat frame = input;
        try {
            if (transposed) {
                frame = fTransposed;
                Core.transpose(input, frame);
            }

            double scale = 1.0;
            int workWidth = WORK_WIDTH;
            if (workWidth > 0 && frame.cols() > workWidth) {
                scale = (double) workWidth / frame.cols();
                Imgproc.resize(frame, fWorking, new Size(workWidth, Math.round(frame.rows() * scale)), 0, 0, Imgproc.INTER_AREA);
            } else {
                frame.copyTo(fWorking);
            }
            Imgproc.cvtColor(fWorking, fBgr, Imgproc.COLOR_RGB2BGR);

            stageNanos.clear();
            prepare(fBgr.cols(), fBgr.rows());
            List<Found> found = run(fBgr);

            double inverseScale = 1.0 / scale;
            List<Ball> balls = new ArrayList<>();
            for (Found f : found) {
                if (!f.kept) continue;
                balls.add(new Ball(f.x * inverseScale, f.y * inverseScale, f.r * inverseScale, f));
            }
            lastTransposed = transposed;
            frameW = frame.cols();
            frameH = frame.rows();
            latest = new Frame(latest.id + 1, Collections.unmodifiableList(balls));
            debugProcessMs = (System.nanoTime() - startTime) / 1e6;
            debugSingles = accepted.size();
            debugSeeds = spotX.length;
            debugConnections = pConns.size();
            debugCircles = pRefitFits.size();
            debugEstimated = pEstimated.size();
            debugKept = balls.size();
            StringBuilder stages = new StringBuilder();
            for (Map.Entry<String, Long> entry : stageNanos.entrySet()) {
                if (stages.length() > 0) stages.append(" | ");
                stages.append(entry.getKey()).append(" ").append(String.format(Locale.US, "%.1f", entry.getValue() / 1e6));
            }
            debugStages = stages.toString();
            if (frameEpoch == epoch) fresh = latest; // last: a reader that sees this frame also sees its debug fields

            boolean publish = fFrameCount == 0;
            fFrameCount = (fFrameCount + 1) % Math.max(1, STREAM_EVERY);
            if (publish) publishView(mode, frame, fBgr);
            debugError = "";
        } catch (RuntimeException error) {
            latest = new Frame(latest.id, Collections.<Ball>emptyList());
            debugError = error.toString();
        } finally {
            releaseTemps();
        }
        return null;
    }

    private void publishView(int mode, Mat frame, Mat bgr) {
        Mat view = fView;
        if (mode <= 0) {
            frame.copyTo(view);
        } else {
            Mat panel = buildPanel(mode);
            Mat rgbPanel = fRgbPanel;
            Imgproc.cvtColor(panel, rgbPanel, Imgproc.COLOR_BGR2RGB);
            if (rgbPanel.cols() != frame.cols() || rgbPanel.rows() != frame.rows()) {
                Imgproc.resize(rgbPanel, view, frame.size(), 0, 0, Imgproc.INTER_NEAREST);
            } else {
                rgbPanel.copyTo(view);
            }
            Imgproc.putText(view, MODE_NAMES[mode] + "  " + panelLabel + String.format(Locale.US, "  %.0f ms", debugProcessMs), new Point(10, 28), FONT, 0.7, new Scalar(255, 255, 0), 2);
        }
        int w = view.cols(), h = view.rows();
        if (fBitmapA == null || fBitmapA.getWidth() != w || fBitmapA.getHeight() != h) {
            fBitmapA = Bitmap.createBitmap(w, h, Bitmap.Config.RGB_565);
            fBitmapB = Bitmap.createBitmap(w, h, Bitmap.Config.RGB_565);
        }
        // write into the buffer that lastFrame does not currently point to
        Bitmap target = fUseB ? fBitmapB : fBitmapA;
        fUseB = !fUseB;
        Utils.matToBitmap(view, target);
        lastFrame.set(target);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        List<Ball> balls = latest.balls;
        boolean transposed = lastTransposed;
        if (balls.isEmpty()) return;

        Paint outline = new Paint();
        outline.setColor(Color.GREEN);
        outline.setStyle(Paint.Style.STROKE);
        outline.setStrokeWidth(scaleCanvasDensity * 3);

        Paint dot = new Paint();
        dot.setColor(Color.RED);
        dot.setStyle(Paint.Style.FILL);

        Paint label = new Paint();
        label.setColor(Color.GREEN);
        label.setTextSize(scaleCanvasDensity * 16);

        int index = 1;
        for (Ball ball : balls) {
            double bx = transposed ? ball.y : ball.x;
            double by = transposed ? ball.x : ball.y;
            float cx = (float) bx * scaleBmpPxToCanvasPx;
            float cy = (float) by * scaleBmpPxToCanvasPx;
            float r = (float) ball.radius * scaleBmpPxToCanvasPx;
            canvas.drawCircle(cx, cy, r, outline);
            canvas.drawCircle(cx, cy, scaleCanvasDensity * 3, dot);

            String text = String.valueOf(index);
            if (ball.hasRange) {
                text += "  " + Math.round(ball.rangeCm) + "cm";
            }
            canvas.drawText(text, cx + r + 4, cy, label);
            index++;
        }
    }
}
