package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vision.Detect;
import org.firstinspires.ftc.teamcode.vision.Pickup;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Press A: one detect frame -> ball pixels -> field x, y -> route, with timings. Vision is off between presses.
 * The robot does not drive; the follower only supplies the pose (starts at Pickup's start pose).
 */
@TeleOp(name = "Vision Test", group = "Tests")
public class VisionTest extends LinearOpMode {
    Follower follower;
    Detect detect;
    final List<String> report = new ArrayList<>();
    int runs = 0;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(Pickup.startX, Pickup.startY, Math.toRadians(Pickup.startHeadingDeg)));
        detect = new Detect(hardwareMap);
        detect.disable();
        report.add("Press A to detect once");

        while (opModeInInit() || opModeIsActive()) {
            follower.update();
            if (gamepad1.aWasPressed()) runOnce();
            show();
        }
        detect.close();
    }

    void runOnce() {
        report.clear();
        runs++;
        Pickup pickup = new Pickup();
        pickup.start();
        long t0 = System.nanoTime();
        detect.enable();
        Detect.Frame frame = null;
        while (frame == null && !isStopRequested() && ms(t0) < Pickup.scanTimeoutMs) {
            follower.update();
            frame = detect.capture();
        }
        detect.disable();
        double waitMs = ms(t0);
        if (frame == null) {
            report.add(String.format(Locale.US, "NO FRAME after %.0f ms (camera not streaming, or processing > scanTimeoutMs)", waitMs));
            if (!detect.debugError.isEmpty()) report.add("Error: " + detect.debugError);
            return;
        }
        Pose pose = follower.pose();
        report.add(String.format(Locale.US, "Detect: %.0f ms processing, %.0f ms enable -> frame", detect.debugProcessMs, waitMs));
        report.add("Stages (ms): " + detect.debugStages);
        report.add(String.format(Locale.US, "Counts: singles %d, seeds %d, circles %d, estimated %d, kept %d",
                detect.debugSingles, detect.debugSeeds, detect.debugCircles, detect.debugEstimated, detect.debugKept));
        if (!detect.debugError.isEmpty()) report.add("Error: " + detect.debugError);
        report.add(String.format(Locale.US, "Colour mask: %.1f%% of image", Detect.debugMaskPct));
        report.add("Centre colour: " + Detect.debugCenter);

        report.add(String.format(Locale.US, "Raw balls: %d (image %dx%d)", frame.balls.size(), Detect.frameW, Detect.frameH));
        for (int i = 0; i < frame.balls.size(); i++) {
            Detect.Ball b = frame.balls.get(i);
            report.add(String.format(Locale.US, " %d: px (%.0f, %.0f) r %.1f%s%s", i + 1, b.x, b.y, b.radius,
                    b.estimated ? " est" : "",
                    b.hasRange ? String.format(Locale.US, ", %.0f cm away", b.rangeCm) : ""));
        }

        long t1 = System.nanoTime();
        pickup.add(frame, pose);
        List<Pose> balls = pickup.positions(pose);
        double imposeMs = ms(t1);
        report.add(String.format(Locale.US, "Field balls: %d (%.1f ms) from robot (%.1f, %.1f, %.0f deg)",
                balls.size(), imposeMs, pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        for (int i = 0; i < balls.size(); i++) {
            Pose b = balls.get(i);
            report.add(String.format(Locale.US, " %d: (%.1f, %.1f) in, %.1f in away, bearing %.0f deg", i + 1, b.x(), b.y(),
                    Math.hypot(b.x() - pose.x(), b.y() - pose.y()), Math.toDegrees(b.heading())));
        }

        long t2 = System.nanoTime();
        Path path = pickup.path(pose);
        double routeMs = ms(t2);
        report.add(String.format(Locale.US, "Route: %.1f ms, %s", routeMs,
                path == null ? "no path" : path.getSegments().size() + " segments"));
        report.add(pickup.debug());
        report.add(String.format(Locale.US, "Total: %.0f ms", ms(t0)));
    }

    void show() {
        telemetry.addData("Runs", runs);
        telemetry.addData("Calibration", Detect.calFromSdk ? "SDK" : "FALLBACK (no SDK calibration at this resolution)");
        telemetry.addData("Pose", follower.pose());
        for (String line : report) telemetry.addLine(line);
        telemetry.update();
    }

    static double ms(long start) {
        return (System.nanoTime() - start) / 1e6;
    }
}
