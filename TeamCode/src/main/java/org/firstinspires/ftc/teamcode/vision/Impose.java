package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayList;
import java.util.List;

/**
 * Camera pixel -> floor point relative to the robot (project), then robot-relative points -> Pedro field
 * coordinates with off-field filtering, dedupe and the cross-frame vote (localize, vote).
 */
public class Impose {
    private final double fx, fy, cx, cy;
    private final double k1, k2, p1, p2, k3;
    private final double camForward, camLeft, camHeight;
    private final double pitch, yaw, ballCenterHeight;

    public Impose(double fx, double fy, double cx, double cy, double calWidth, double calHeight,
                           double k1, double k2, double p1, double p2, double k3,
                           double frameWidth, double frameHeight,
                           double camForwardIn, double camLeftIn, double camHeightIn,
                           double pitchDownDeg, double yawLeftDeg, double ballCenterHeightIn) {
        // same calibration handling as Detect.loadCalibration()
        if ((calWidth > calHeight) != (frameWidth > frameHeight)) {
            double t = fx; fx = fy; fy = t;
            t = cx; cx = cy; cy = t;
            t = p1; p1 = p2; p2 = t;
            t = calWidth; calWidth = calHeight; calHeight = t;
        }
        this.fx = fx * frameWidth / calWidth;
        this.cx = cx * frameWidth / calWidth;
        this.fy = fy * frameHeight / calHeight;
        this.cy = cy * frameHeight / calHeight;
        this.k1 = k1;
        this.k2 = k2;
        this.p1 = p1;
        this.p2 = p2;
        this.k3 = k3;
        this.camForward = camForwardIn;
        this.camLeft = camLeftIn;
        this.camHeight = camHeightIn;
        this.pitch = Math.toRadians(pitchDownDeg);
        this.yaw = Math.toRadians(yawLeftDeg);
        this.ballCenterHeight = ballCenterHeightIn;
    }

    // iterative inverse of the OpenCV distortion model, returns {xn, yn}
    double[] undistort(double u, double v) {
        double x0 = (u - cx) / fx, y0 = (v - cy) / fy;
        double x = x0, y = y0;
        for (int i = 0; i < 20; i++) {
            double r2 = x * x + y * y;
            double icdist = 1 / (1 + ((k3 * r2 + k2) * r2 + k1) * r2);
            double dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
            double dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
            x = (x0 - dx) * icdist;
            y = (y0 - dy) * icdist;
        }
        return new double[]{x, y};
    }

    // returns {forward, left} in inches, or null if the ray misses the ball plane
    public double[] project(double u, double v) {
        double[] n = undistort(u, v);
        // camera axes in robot frame: forward, left, up
        double f = 1, l = -n[0], z = -n[1];

        double cp = Math.cos(pitch), sp = Math.sin(pitch);
        double f1 = f * cp + z * sp;
        double z1 = -f * sp + z * cp;

        double cyaw = Math.cos(yaw), syaw = Math.sin(yaw);
        double f2 = f1 * cyaw - l * syaw;
        double l2 = f1 * syaw + l * cyaw;

        if (z1 >= -1e-9) return null;
        double t = (ballCenterHeight - camHeight) / z1;
        if (t <= 0) return null;
        return new double[]{camForward + t * f2, camLeft + t * l2};
    }

    // ---- robot-relative -> field ----
    public static double fieldSize = 141.5, ballRadius = 1.425, ballDiameter = 2.85;

    public static List<double[]> localize(List<double[]> robotPoints, double rx, double ry, double heading) {
        return localize(robotPoints, rx, ry, heading, fieldSize, ballRadius, ballDiameter);
    }

    public static List<double[]> localize(List<double[]> robotPoints, double rx, double ry, double heading,
                                          double size, double margin, double diameter) {
        return merge(filter(toField(robotPoints, rx, ry, heading), size, margin), diameter);
    }

    // robot {forward, left} -> field {x, y}; heading is CCW radians, 0 = +x
    static List<double[]> toField(List<double[]> robotPoints, double rx, double ry, double heading) {
        double c = Math.cos(heading), s = Math.sin(heading);
        List<double[]> out = new ArrayList<>();
        for (double[] p : robotPoints) {
            out.add(new double[]{rx + p[0] * c - p[1] * s, ry + p[0] * s + p[1] * c});
        }
        return out;
    }

    static List<double[]> filter(List<double[]> pts, double size, double margin) {
        List<double[]> out = new ArrayList<>();
        for (double[] p : pts) {
            if (p[0] >= margin && p[0] <= size - margin && p[1] >= margin && p[1] <= size - margin) out.add(p);
        }
        return out;
    }

    // Votes field points over several frames: each point joins the nearest cluster within matchRadius, at most one point per
    // cluster per frame (touching balls stay apart). Clusters seen in fewer than minFrames frames (flicker, glare) are dropped;
    // the rest become their average, ordered by first sighting.
    public static List<double[]> vote(List<List<double[]>> frames, double matchRadius, int minFrames) {
        List<double[]> sums = new ArrayList<>(); // {sum x, sum y, frames}
        for (List<double[]> frame : frames) {
            boolean[] taken = new boolean[sums.size()]; // clusters made by this frame are not in here, so it can't match them
            for (double[] p : frame) {
                int best = -1;
                double bestD = matchRadius;
                for (int c = 0; c < taken.length; c++) {
                    if (taken[c]) continue;
                    double[] s = sums.get(c);
                    double d = Math.hypot(p[0] - s[0] / s[2], p[1] - s[1] / s[2]);
                    if (d <= bestD) {
                        bestD = d;
                        best = c;
                    }
                }
                if (best < 0) {
                    sums.add(new double[]{p[0], p[1], 1});
                } else {
                    double[] s = sums.get(best);
                    s[0] += p[0];
                    s[1] += p[1];
                    s[2]++;
                    taken[best] = true;
                }
            }
        }
        List<double[]> out = new ArrayList<>();
        for (double[] s : sums) if (s[2] >= minFrames) out.add(new double[]{s[0] / s[2], s[1] / s[2]});
        return out;
    }

    // transitive clustering; each cluster becomes its average, ordered by first member
    static List<double[]> merge(List<double[]> pts, double diameter) {
        int n = pts.size();
        boolean[] used = new boolean[n];
        double d2 = diameter * diameter;
        List<double[]> out = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            if (used[i]) continue;
            used[i] = true;
            List<Integer> todo = new ArrayList<>();
            todo.add(i);
            double sx = 0, sy = 0;
            int count = 0;
            while (!todo.isEmpty()) {
                int a = todo.remove(todo.size() - 1);
                double[] pa = pts.get(a);
                sx += pa[0];
                sy += pa[1];
                count++;
                for (int j = 0; j < n; j++) {
                    if (used[j]) continue;
                    double[] pj = pts.get(j);
                    double dx = pa[0] - pj[0], dy = pa[1] - pj[1];
                    if (dx * dx + dy * dy < d2) {
                        used[j] = true;
                        todo.add(j);
                    }
                }
            }
            out.add(new double[]{sx / count, sy / count});
        }
        return out;
    }
}
