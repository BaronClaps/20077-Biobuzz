package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Detect frames in, ball positions and a Pedro path out:
 * start(); add(detect.capture(), pose) every loop until frames() >= scanFrames; positions(pose); path(pose).
 * Camera settings and mount live in Detect. Inches, degrees unless noted.
 */
@Config
public class Pickup {
    // ---- robot ----
    public static double robotLength = 14, robotWidth = 12.5;
    public static double intakeWidth = 10; // usable width: two balls are taken together when their gap is within intakeWidth - ballDiameter
    public static double intakeReach = 7; // robot center to where the intake grabs: half the 14 in robot length

    // ---- planner ----
    public static double ballDiameter = 2.85, ballCenterHeight = 1.425;
    public static double turnPenalty = 6.0; // cost per radian of heading change when choosing the ball order
    public static int maxBalls = 24; // only the nearest this many detections go to the planner (bounds the planning time)
    public static int capacity = 4; // balls the intake holds: the user's stated number, not a placeholder
    // Keep-in bound (inches, Pedro coordinates, field is 141.5 x 141.5): the WHOLE robot body (robotLength x robotWidth at the
    // heading it actually has) stays inside [boundMin + boundClearance, boundMax - boundClearance] on every point of the route.
    // Default = the whole field. e.g. boundMaxX = 70.75 keeps the robot on the low-x half. Balls the planner cannot collect with
    // the body inside the bound are skipped; a start pose already outside the bound gives an empty plan.
    // Keep boundClearance >= 0.5: the route is checked at its samples and the clearance absorbs the sag between them.
    public static double boundMinX = 0, boundMinY = 0, boundMaxX = 141.5, boundMaxY = 141.5;
    public static double boundClearance = 0.5;

    // ---- route ----
    public static double runIn = 6; // TUNE: straight inches before the ball reaches the intake, so the robot is square on
    public static double driveThrough = 2; // robot fact: must drive >= 2 in past each ball for a sure pickup (straight, past the ball)
    public static double rMin = 5; // TUNE: tightest turn radius (in) the route may use
    public static double approachSpeed = 0.5; // TUNE: fraction of vmax on the straight run-ins and slow pieces

    // The planner's MODEL of the robot: the route is checked (no backing up, heading reaches the run-in heading in time) by
    // simulating a robot at these numbers, so the robot must really drive at them. path() enforces vmax on the Pedro
    // paths with maxVelocityConstraint (in/s, Foresight.coast): fast pieces are capped at vmax, slow pieces at
    // vmax * approachSpeed. Without the cap Constants.foresightConfig would drive ~79 in/s (maxAchievableForwardVelocity)
    // against the 40 modelled here, and the verified no-backing-up guarantee would not hold.
    // The no-backing-up guarantee also rests on these Pedro behaviours, none of which the planner can enforce:
    //  - heading: Pedro's heading control is a plain P loop on the interpolated heading, not rate limited, so the robot is
    //    assumed to track it at least as fast as turnRateDeg; a piece is accepted only if the simulated heading ends within
    //    END_LAG of the run-in heading and never points more than ~83 deg off the path direction.
    //  - drift: the localizer stays within a couple of inches, so the path corrections stay small.
    //  - speed: the vmax cap above, and the robot able to hold it around the tightest accepted curve (rMin).
    // If the robot backs up or crabs on a curve, lower vmax / turnRateDeg or raise rMin before anything else.
    // TUNE: accel and turnRateDeg are not enforced by Pedro; measure them on the robot and keep them at or below what it
    // really does (lower = safer, the planner then accepts fewer sharp turns). Keep vmax at or below the real top speed.
    public static double vmax = 40, accel = 80; // in/s, in/s^2
    public static double turnRateDeg = 270; // deg/s

    // ---- scan / auto ----
    public static double settleMs = 500; // wait before each scan so vision settles
    public static int scanFrames = 1; // frames voted on per scan; try 3 if one-frame glare or missed balls show up
    public static double scanMinSeen = 0.5; // a ball must be in at least this fraction of them
    public static double scanMatchRadius = 2; // in: same ball across frames if within this
    public static double scanTimeoutMs = 2500; // plan with the frames seen so far after this; keep about 2x the Vision Test frame time
    public static double finishWaitMs = 1000; // max wait for the follower to stop after a pass
    public static double passTimeoutMs = 15000;
    public static int maxPasses = 5;
    public static double missRadius = 3; // a ball this close to an earlier target counts as missed
    public static double startX = 72, startY = 8, startHeadingDeg = 90; // PLACEHOLDER: real start pose

    private final Route.Params params = new Route.Params();
    private Impose impose;
    private List<double[]> balls = new ArrayList<>();
    private Route.Plan plan = null;
    private List<double[]> targets = new ArrayList<>(); // balls the last plan went for
    private List<double[]> others = new ArrayList<>(); // balls the last plan left alone
    private int missed = 0, collected = 0;
    private final List<List<double[]>> scanned = new ArrayList<>();
    private long lastFrameId = -1;

    public Pickup() {
        rebuild();
    }

    // takes the current settings (dashboard edits, the SDK calibration once the camera runs); start() calls it
    public void rebuild() {
        // Detect holds the SDK's webcam calibration once the camera has started; before that its fallback
        double[] k = Detect.calK, d = Detect.calD;
        double calW = Detect.calW, calH = Detect.calH;
        double frameW = Detect.frameW, frameH = Detect.frameH;
        if (frameW <= 0) {
            frameW = calW;
            frameH = calH;
        }
        impose = new Impose(k[0], k[4], k[2], k[5], calW, calH,
                d[0], d[1], d[2], d[3], d.length > 4 ? d[4] : 0,
                frameW, frameH,
                Detect.camForwardIn, Detect.camLeftIn, Detect.camHeightIn,
                Detect.camPitchDownDeg, Detect.camYawLeftDeg, ballCenterHeight);
        Route.Params p = params;
        p.intakeWidth = intakeWidth;
        p.intakeReach = intakeReach;
        p.robotLength = robotLength;
        p.robotWidth = robotWidth;
        p.ballDiameter = ballDiameter;
        p.turnPenalty = turnPenalty;
        p.boundMinX = boundMinX;
        p.boundMinY = boundMinY;
        p.boundMaxX = boundMaxX;
        p.boundMaxY = boundMaxY;
        p.boundClearance = boundClearance;
        p.runIn = runIn;
        p.driveThrough = driveThrough;
        p.rMin = rMin;
        p.approachSpeed = approachSpeed;
        p.vmax = vmax;
        p.accel = accel;
        p.turnRate = turnRateDeg * Math.PI / 180; // not Math.toRadians: keeps the numbers equal to the JS reference
    }

    // ---- scan ----

    /** Begin a scan: fresh settings, no frames. Call before detect.enable(). */
    public void start() {
        rebuild();
        scanned.clear();
        lastFrameId = -1;
    }

    /** Call every loop with detect.capture(); keeps the frame if it is a new one. Returns true when it did. */
    public boolean add(Detect.Frame frame, Pose robotPose) {
        if (frame == null || frame.id == lastFrameId) return false;
        lastFrameId = frame.id;
        scanned.add(localize(frame.balls, robotPose));
        return true;
    }

    public int frames() {
        return scanned.size();
    }

    /**
     * Ends the scan: the balls seen in at least scanMinSeen of the frames, as field Poses (Pedro inches). Heading = bearing
     * from robotPose to the ball. Also works out which of the last path's targets were collected or missed.
     * No frame since start() (timeout, camera hiccup): returns an empty list and counts nothing, since the
     * targets were not seen gone, just not looked at.
     */
    public List<Pose> positions(Pose robotPose) {
        if (scanned.isEmpty()) return new ArrayList<>();
        int minFrames = Math.max(1, (int) Math.ceil(scanMinSeen * scanned.size()));
        accept(Impose.vote(scanned, scanMatchRadius, minFrames), robotPose);
        List<Pose> out = new ArrayList<>();
        for (double[] b : balls) out.add(new Pose(b[0], b[1], Math.atan2(b[1] - robotPose.y(), b[0] - robotPose.x())));
        return out;
    }

    private List<double[]> localize(List<Detect.Ball> detected, Pose robotPose) {
        List<double[]> local = new ArrayList<>();
        for (Detect.Ball b : detected) {
            double[] p = impose.project(b.x, b.y);
            if (p != null) local.add(p);
        }
        return Impose.localize(local, robotPose.x(), robotPose.y(), robotPose.heading(),
                Impose.fieldSize, ballDiameter / 2, ballDiameter);
    }

    private void accept(List<double[]> field, Pose robotPose) {
        // a target seen again next to where it was is a miss; the rest of the targets are in the intake.
        // One seen ball explains at most one target, and a seen ball that is really an earlier non-target neighbour
        // (balls can touch, closer than missRadius) is not a miss.
        boolean[] claimed = new boolean[field.size()];
        List<double[]> order = new ArrayList<>(); // {target index, ball index, distance}
        for (int ti = 0; ti < targets.size(); ti++) {
            double[] t = targets.get(ti);
            for (int bi = 0; bi < field.size(); bi++) {
                double[] b = field.get(bi);
                double d = Math.hypot(b[0] - t[0], b[1] - t[1]);
                if (d > missRadius) continue;
                boolean neighbour = false;
                for (double[] o : others) if (Math.hypot(b[0] - o[0], b[1] - o[1]) < d) neighbour = true;
                if (!neighbour) order.add(new double[] {ti, bi, d});
            }
        }
        java.util.Collections.sort(order, new java.util.Comparator<double[]>() {
            @Override
            public int compare(double[] p, double[] q) {
                return Double.compare(p[2], q[2]);
            }
        });
        boolean[] targetMissed = new boolean[targets.size()];
        missed = 0;
        for (double[] m : order) {
            int ti = (int) m[0], bi = (int) m[1];
            if (targetMissed[ti] || claimed[bi]) continue;
            targetMissed[ti] = true;
            claimed[bi] = true;
            missed++;
        }
        collected += targets.size() - missed;
        others = new ArrayList<>();
        targets = new ArrayList<>();
        // pathological ball counts: keep only the nearest few, the route is for the best 4 anyway
        final double rx = robotPose.x(), ry = robotPose.y();
        if (field.size() > maxBalls) {
            java.util.Collections.sort(field, new java.util.Comparator<double[]>() {
                @Override
                public int compare(double[] p, double[] q) {
                    return Double.compare(Math.hypot(p[0] - rx, p[1] - ry), Math.hypot(q[0] - rx, q[1] - ry));
                }
            });
            field = new ArrayList<>(field.subList(0, maxBalls));
        }
        balls = field;
        plan = null;
    }

    // ---- plan + path ----

    /** Plans from start through the best balls of the last positions() (room left in the intake), as one Pedro path; null if none. */
    public Path path(Pose start) {
        return path(plan(start), start);
    }

    // plans only for the room left in the intake
    public Route.Plan plan(Pose start) {
        params.capacity = remaining();
        plan = Route.plan(start.x(), start.y(), start.heading(), balls, params);
        targets = new ArrayList<>();
        others = new ArrayList<>();
        boolean[] chosen = new boolean[balls.size()];
        for (int id : plan.chosen) {
            targets.add(balls.get(id));
            chosen[id] = true;
        }
        for (int i = 0; i < balls.size(); i++) if (!chosen[i]) others.add(balls.get(i));
        return plan;
    }

    // one compound path through every stop, or null if there is nothing to drive. Bezier pieces take their heading from the
    // tangent or linear(h0, h1, endT); the straight run-ins keep the stop heading. No holds, turns or stops in between.
    public Path path(Route.Plan plan, Pose start) {
        List<Route.Segment> recs = Route.buildSegments(plan, start.x(), start.y(), start.heading(), params);
        List<Path> segs = new ArrayList<>();
        for (Route.Segment s : recs) {
            // Pedro's Line throws on zero length; a piece this short is already inside the previous one's end
            if (Math.hypot(s.x1 - s.x0, s.y1 - s.y0) < 1e-3) continue;
            Path p;
            if (s.mode == Route.Mode.CURVE) {
                // Bezier ignores pose headings; the poses just carry the points
                Path c = Paths.curve(new Pose(s.x0, s.y0, s.h0), new Pose(s.cx1, s.cy1, s.h0),
                        new Pose(s.cx2, s.cy2, s.h1), new Pose(s.x1, s.y1, s.h1));
                p = s.headingEndT > 0 ? c.linear(s.h0, s.h1, s.headingEndT) : c.tangent();
            } else {
                p = Paths.line(new Pose(s.x0, s.y0, s.h0), new Pose(s.x1, s.y1, s.h1))
                        .constant(new Pose(s.x1, s.y1, s.h1));
            }
            segs.add(capped(p, s.slow));
        }
        if (segs.isEmpty()) return null;
        if (segs.size() == 1) return segs.get(0);
        return Paths.path(segs.toArray(new Path[0]));
    }

    // Pedro 3 Foresight.coast: targetVel = min(maxAchievable, maxPathSpeed * maxAchievable, maxVelocityConstraint). maxPathSpeed is
    // a fraction of the direction-dependent maxAchievableVelocity (79 forward, 59 strafe here), so it cannot give the planner's
    // fixed in/s; maxVelocityConstraint is in/s. Every piece is capped so the robot drives the speeds the planner verified.
    private Path capped(Path p, boolean slow) {
        double v = vmax * (slow ? approachSpeed : 1);
        return p.with(Constants.foresightConfig.maxVelocityConstraint.at(v));
    }

    // ---- state ----

    public List<double[]> balls() {
        return balls;
    }

    public int missed() {
        return missed;
    }

    public int collected() {
        return collected;
    }

    public int remaining() {
        return Math.max(0, capacity - collected);
    }

    public boolean full() {
        return remaining() == 0;
    }

    public String debug() {
        if (balls.isEmpty()) return "Balls: none";
        if (plan == null) return "Balls: " + balls.size() + " seen, no plan";
        return Route.describe(plan, balls);
    }
}
