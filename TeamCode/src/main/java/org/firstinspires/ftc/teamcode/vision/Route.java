package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * One continuous forward route through the best balls (up to capacity), straight-on into each one. Pure math, no Pedro types.
 * Port of experiments/variants/arc_final.js; keep the two in step. All budgets count work units, never time, so the same
 * inputs give the same route on any machine. StrictMath (fdlibm) is used for trig so the numbers match the JS reference.
 */
public class Route {
    public enum Mode {CURVE, CONSTANT}

    /** Tunables and robot facts. Defaults match the JS reference; Pickup copies its live values in (e.g. approachSpeed 0.75). */
    public static class Params {
        public int capacity = 4;
        public double intakeWidth = 10, intakeReach = 7, ballDiameter = 2.85, turnPenalty = 6.0;
        public double robotLength = 14, robotWidth = 12.5;
        // keep-in bound (contract v5): the whole body (robotLength x robotWidth at the heading it has) stays inside
        // [boundMin + boundClearance, boundMax - boundClearance] everywhere on the route, at every tier. Default = whole field.
        // Invalid values (not finite, outside [0, FIELD], min >= max - robotWidth) fall back to the whole field; clearance is clamped to [0, 10].
        // The path is checked at its samples: keep boundClearance >= 0.5 so the sag between samples stays inside the real bound.
        public double boundMinX = 0, boundMinY = 0, boundMaxX = FIELD, boundMaxY = FIELD, boundClearance = 0.5;
        public double approachSpeed = 0.5; // fraction of vmax on slow pieces (Pickup caps them with maxVelocityConstraint)
        public double accel = 80, vmax = 40, turnRate = 270 * Math.PI / 180; // heading simulation; rad/s

        // route geometry
        public double runIn = 6; // straight inches before the ball reaches the intake centre
        public double driveThrough = 2; // straight inches carried on past the contact pose; robot fact: >= 2 in past each ball for a sure pickup
        public double rMin = 5; // tier 0 minimum radius of curvature; tier 1 uses 0.7x, tier 2 0.5x
        public double lateral = 3.3; // balls may sit this far off the intake centre line (usable half width 5 - ball radius 1.425, minus margin)
        public double chainTol = 3.3; // a later single may share the previous straight line if this close to it
        public double directStraight = 4.6; // robot already on the first line may turn and drive if this much straight remains

        // cost weights (inch equivalents)
        public double collateralW = 70, shortW = 14, curvW = 45, startTurnW = 3, pieceCollW = 30, corridorW = 20;
        public double corridorHalf = 5.6, earlyW = 2, clipW = 30;
        public double crossB = 6; // full-route check: per self-crossing of the route (counted like a swept ball, so stages keep looking)
        public double startTanMax = 0.785; // first tangent at most this far (rad) from the robot heading; the heading itself starts at the robot heading
        // a piece turning its path direction by more than loopRad beyond its net turn costs loopW per rad beyond and makes the route not clean
        public double loopRad = 1.75, loopW = 1000;

        // search breadth
        public int topM = 120, perSubset = 3, maxDp = 40, wantFeasible = 6, wantFeasible2 = 2, beam = 5;
        public int samples = 10, kCheckN = 60;
        public double kTol = 1.2;

        // via (two-cubic U-turn) pieces, only tried when a single cubic fails
        public double[] viaR = {6.5}, viaR2 = {5, 9}, viaTh = {1.0, 2.0, 3.0, 4.0};
        public double viaLag = 0.08, viaPen = 14, viaSlowW = 0.8;
        public int viaTries = 12;
        public double pruneA = 4.6, pruneB = -1.5;

        // work budgets: {tier, via (0|1), cap}; rescue stages only run while no route exists at all
        public double[][] stages = {{0, 0, 90000}, {1, 0, 50000}, {0, 1, 30000}, {1, 1, 150000}};
        public double[][] rescue = {{2, 0, 220000}, {2, 1, 140000}, {1, 1, 350000}, {0, 1, 350000}};
        public double hardWork = 220000, dpOvershoot = 1.25;
        public int maxTier2 = 60;
    }

    public static class Stop {
        public final double x, y, heading;
        public final int[] balls;

        Stop(double x, double y, double heading, int[] balls) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.balls = balls;
        }

        public boolean pair() {
            return balls.length == 2;
        }
    }

    /** Per-stop route data: run-in start (ax, ay), end of drive-through (ex, ey), stop heading h, and the piece leading in. */
    public static class RouteItem {
        public final double ax, ay, ex, ey, h, hs, f1, f2, endT, dEndT;
        public final boolean merge, direct, pslow;
        public final Via via;

        RouteItem(State s) {
            ax = s.ax;
            ay = s.ay;
            ex = s.ex;
            ey = s.ey;
            h = s.h;
            merge = s.merge;
            hs = s.hs;
            direct = s.direct;
            dEndT = s.dEndT;
            f1 = s.piece != null ? s.piece.f1 : 0;
            f2 = s.piece != null ? s.piece.f2 : 0;
            endT = s.piece != null ? s.piece.endT : 0;
            pslow = s.piece != null && s.piece.slow;
            via = s.piece != null ? s.piece.via : null;
        }
    }

    public static class Plan {
        public final List<Stop> stops;
        public final double cost, work;
        public final int[] chosen; // ball indices collected, sorted
        public final List<RouteItem> route; // empty when there is nothing to drive
        public final int tier;
        public final String reason; // why the plan is empty ("" when it is not)

        Plan(List<Stop> stops, double cost, List<RouteItem> route, int tier, double work, String reason) {
            this.reason = reason;
            this.stops = stops;
            this.cost = cost;
            this.route = route;
            this.tier = tier;
            this.work = work;
            int n = 0;
            for (Stop s : stops) n += s.balls.length;
            chosen = new int[n];
            int k = 0;
            for (Stop s : stops) for (int id : s.balls) chosen[k++] = id;
            Arrays.sort(chosen);
        }

        // no route found (or no balls): nothing to drive, never a backing-up substitute
        public boolean empty() {
            return stops.isEmpty();
        }
    }

    public static class Segment {
        public final Mode mode;
        public final double x0, y0, x1, y1, h0, h1;
        public final boolean slow;
        public final int stop;
        public final double cx1, cy1, cx2, cy2; // control points, curves only
        public final double headingEndT; // curves only: 0 = tangent heading, else linear(h0, h1) finished at this fraction

        Segment(Mode mode, double x0, double y0, double x1, double y1, double h0, double h1, boolean slow, int stop,
                double cx1, double cy1, double cx2, double cy2, double headingEndT) {
            this.mode = mode;
            this.x0 = x0;
            this.y0 = y0;
            this.x1 = x1;
            this.y1 = y1;
            this.h0 = h0;
            this.h1 = h1;
            this.slow = slow;
            this.stop = stop;
            this.cx1 = cx1;
            this.cy1 = cy1;
            this.cx2 = cx2;
            this.cy2 = cy2;
            this.headingEndT = headingEndT;
        }
    }

    // ---- constants of the piece model ----
    public static final double FIELD = 141.5; // contract v5
    // handle fractions of the chord tried for a cubic: COARSE first, LOOPS (long handles) only if none works
    private static final double[][] COARSE = {{0.35, 0.35}, {0.6, 0.6}, {0.35, 0.8}, {0.8, 0.35}};
    private static final double[][] LOOPS = {{0.9, 0.9}, {1.3, 0.6}, {0.6, 1.3}, {1.3, 1.3}};
    private static final double[][] VIA_F = {{0.35, 0.35}, {0.6, 0.6}, {0.35, 0.8}, {0.8, 0.35}, {0.9, 0.9}, {1.3, 0.6}};
    // heading modes per geometry, cheapest first: endT (0 = tangent), slow, cost penalty
    private static final double[] MODE_END_T = {0, 0.85, 0, 0.85};
    private static final boolean[] MODE_SLOW = {false, false, true, true};
    private static final double[] MODE_PEN = {0, 0.8, 6, 6.8};
    private static final double REV_LIM = 1.45; // max |path direction - heading| on a piece (~83 deg): no backing up
    private static final double END_LAG = 0.2; // heading must converge to the run-in heading this closely at a piece end
    private static final double PEAK_W = 25; // weight of peak curvature in a piece cost
    private static final double PIVOT_TOL = 0.02; // first tangent within this of the robot heading: tangent heading starts where the robot is
    private static final double DIRECT_TOL = 0.01; // direct start: heading already this close to the stop heading -> constant heading
    private static final double[] DIRECT_FR = {1.0, 0.75, 0.5}; // direct start: fractions of the available straight for the rotation
    private static final int MAX_SAMPLES = 40; // scratch size, at least refine samples + 1
    private static final double BOUND_STEP = 1.0; // gapIn re-check spacing: inches a body corner may move between two dense bound checks

    // ---- per-plan state ----

    // work counter and scratch buffers (two slots: via pieces keep two geometries alive)
    static class Work {
        double work = 0, base = 0, limit = Double.POSITIVE_INFINITY, hEnd = 0;
        final double[][] S = new double[2][MAX_SAMPLES], D = new double[2][MAX_SAMPLES], E = new double[2][MAX_SAMPLES];
        final double[][] X = new double[2][MAX_SAMPLES], Y = new double[2][MAX_SAMPLES]; // sample positions
    }

    static class Pose3 {
        double x, y, h;

        Pose3(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }
    }

    public static class Via {
        final double x, y, h, f1, f2;

        Via(double x, double y, double h, double f1, double f2) {
            this.x = x;
            this.y = y;
            this.h = h;
            this.f1 = f1;
            this.f2 = f2;
        }
    }

    static class Piece {
        final double cost, f1, f2, len, endT, coll, lx; // lx = loop excess: tangent rotation beyond the turn the hop needs
        final boolean slow;
        final Via via;

        Piece(double cost, double f1, double f2, double len, double endT, boolean slow, Via via, double coll, double lx) {
            this.lx = lx;
            this.cost = cost;
            this.f1 = f1;
            this.f2 = f2;
            this.len = len;
            this.endT = endT;
            this.slow = slow;
            this.via = via;
            this.coll = coll;
        }
    }

    // sampled cubic, samples live in Work.S/D/E[slot]
    static class Geom {
        final double f1, f2, len, cost, h1, tr;
        final double x0, y0, cx1, cy1, cx2, cy2, x1, y1; // control points (segment naming) for gapIn
        final int n, slot;

        Geom(double f1, double f2, double len, int n, int slot, double tr, double cost, double h1,
             double x0, double y0, double cx1, double cy1, double cx2, double cy2, double x1, double y1) {
            this.x0 = x0;
            this.y0 = y0;
            this.cx1 = cx1;
            this.cy1 = cy1;
            this.cx2 = cx2;
            this.cy2 = cy2;
            this.x1 = x1;
            this.y1 = y1;
            this.tr = tr;
            this.f1 = f1;
            this.f2 = f2;
            this.len = len;
            this.n = n;
            this.slot = slot;
            this.cost = cost;
            this.h1 = h1;
        }
    }

    static final double BOUND_EPS = 1e-9;

    /** Sanitised keep-in bound; x0..y1 are the INNER limits the body must stay within (bound +- clearance). */
    public static class Bound {
        public final double minX, minY, maxX, maxY, clearance, x0, y0, x1, y1;

        Bound(double minX, double minY, double maxX, double maxY, double clearance) {
            this.minX = minX;
            this.minY = minY;
            this.maxX = maxX;
            this.maxY = maxY;
            this.clearance = clearance;
            x0 = minX + clearance;
            y0 = minY + clearance;
            x1 = maxX - clearance;
            y1 = maxY - clearance;
        }
    }

    static boolean finite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    public static Bound sanitizeBound(Params p) {
        double a = p.boundMinX, b = p.boundMinY, c = p.boundMaxX, d = p.boundMaxY, cl = p.boundClearance;
        boolean ok = finite(a) && finite(b) && finite(c) && finite(d);
        if (ok) ok = a >= 0 && b >= 0 && c <= FIELD && d <= FIELD && a < c - p.robotWidth && b < d - p.robotWidth;
        if (!ok) {
            a = 0;
            b = 0;
            c = FIELD;
            d = FIELD;
        }
        cl = finite(cl) ? Math.min(10, Math.max(0, cl)) : 0.5;
        return new Bound(a, b, c, d, cl);
    }

    /** Whole-body test: the robotLength x robotWidth rectangle at (x, y, h) lies inside the inner bound (all four corners). */
    static boolean bodyIn(Bound bd, double hl, double hw, double x, double y, double h) {
        double ca = Math.abs(StrictMath.cos(h)), sa = Math.abs(StrictMath.sin(h));
        double ex = ca * hl + sa * hw, ey = sa * hl + ca * hw;
        return x - ex >= bd.x0 && x + ex <= bd.x1 && y - ey >= bd.y0 && y + ey <= bd.y1;
    }

    // Same test as a distance: how far (in) the body is inside the inner bound at its closest side (< 0 = outside).
    static double bodySlack(Bound bd, double hl, double hw, double x, double y, double h) {
        double ca = Math.abs(StrictMath.cos(h)), sa = Math.abs(StrictMath.sin(h));
        double ex = ca * hl + sa * hw, ey = sa * hl + ca * hw;
        return Math.min(Math.min(x - ex - bd.x0, bd.x1 - x - ex), Math.min(y - ey - bd.y0, bd.y1 - y - ey));
    }

    // search context per (tier, via on/off); the memo is shared between the plain and via context of one tier
    static class Ctx {
        final Params p;
        final int tier;
        final Work ws;
        final double[][] balls;
        final boolean via;
        double viaLimit = 0;
        final Map<String, Piece> memo;
        final double L, d, reach, slowF, kmax, curvW, A, W, vFast, vSlow, vJ;
        final double[] Lopts, lo, hi;
        final Bound bd;
        final double hl, hw; // half robot length / width
        final int samples;
        final Set<String> dead = new HashSet<String>(); // ball-prefix keys proven infeasible
        final Set<String> badHop = new HashSet<String>(); // "prev>next" hops proven infeasible

        Ctx(Params p, int tier, Work ws, double[][] balls, Map<String, Piece> memo, boolean via, Bound bd) {
            double hmin = Math.min(p.robotLength, p.robotWidth) / 2 - BOUND_EPS; // centre prefilter: the body is at least this wide
            this.bd = bd;
            hl = p.robotLength / 2;
            hw = p.robotWidth / 2;
            this.p = p;
            this.tier = tier;
            this.ws = ws;
            this.balls = balls;
            this.via = via;
            this.memo = memo;
            L = p.runIn;
            Lopts = tier == 2 ? new double[] {p.runIn, 5.3, 3.5} : new double[] {p.runIn};
            d = p.driveThrough;
            reach = p.intakeReach;
            slowF = 1 / Math.max(0.05, p.approachSpeed);
            lo = new double[] {bd.x0 + hmin, bd.y0 + hmin};
            hi = new double[] {bd.x1 - hmin, bd.y1 - hmin};
            kmax = tier == 0 ? 1 / p.rMin : tier == 1 ? 1 / (p.rMin * 0.7) : 1 / (p.rMin * 0.5);
            samples = p.samples;
            curvW = p.curvW;
            A = p.accel;
            W = p.turnRate;
            vFast = p.vmax;
            vSlow = p.vmax * p.approachSpeed;
            vJ = p.vmax * p.approachSpeed;
        }

        boolean bodyIn(double x, double y, double h) {
            return Route.bodyIn(bd, hl, hw, x, y, h);
        }

        double bodySlack(double x, double y, double h) {
            return Route.bodySlack(bd, hl, hw, x, y, h);
        }
    }

    // a candidate stop: single ball (b = -1) or a pair taken together, driving perpendicular to the pair axis
    static class Group {
        final int a, b, size;
        final double x, y, perp, alt;
        List<double[]> feas = new ArrayList<double[]>(); // [heading, offset] poses straight-on feasible at the full run-in (groupReachable)

        Group(int a, int b, double x, double y, double perp, double alt) {
            this.a = a;
            this.b = b;
            size = b < 0 ? 1 : 2;
            this.x = x;
            this.y = y;
            this.perp = perp;
            this.alt = alt;
        }
    }

    private static class Seq {
        final double cost;
        final Group[] seq;

        Seq(double cost, Group[] seq) {
            this.cost = cost;
            this.seq = seq;
        }
    }

    // one DP state = a stop with heading h, lateral offset o and run-in length L:
    // k = intake-centre contact pose, a = run-in start (k - (L+s)*dir), e = end of drive-through (k + (d+s)*dir).
    // A pair may be taken at any heading where both balls fit across the intake; s = half the pair's spread along the heading, so the
    // near ball still gets the full run-in and the far ball the full drive-through (s = 0 for singles and pairs taken perpendicular).
    private static class State {
        final Group g;
        final double h, o, kx, ky, L, s, ax, ay, ex, ey;
        final State chainOf;
        double cost = Double.POSITIVE_INFINITY, hs = 0, dEndT = 0;
        State prev = null;
        Piece piece = null;
        boolean merge = false, direct = false;

        State(Group g, double h, double o, State chainOf, double kx, double ky, double L, double s, double ax, double ay, double ex,
              double ey) {
            this.s = s;
            this.g = g;
            this.h = h;
            this.o = o;
            this.chainOf = chainOf;
            this.kx = kx;
            this.ky = ky;
            this.L = L;
            this.ax = ax;
            this.ay = ay;
            this.ex = ex;
            this.ey = ey;
        }
    }

    private static class Cand {
        final double h, o, L;
        final State chain;

        Cand(double h, double o, State chain, double L) {
            this.h = h;
            this.o = o;
            this.chain = chain;
            this.L = L;
        }
    }

    private static class Dp {
        double cost, coll;
        int loops = 0; // pieces whose loop excess is above loopRad
        final List<State> items;

        Dp(double cost, List<State> items) {
            this.cost = cost;
            this.items = items;
        }
    }

    // ---- helpers ----

    static double wrap(double a) {
        a = a % (2 * Math.PI);
        if (a > Math.PI) a -= 2 * Math.PI;
        else if (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static double dirOf(double ax, double ay, double bx, double by) {
        return StrictMath.atan2(by - ay, bx - ax);
    }

    private static double dist(double ax, double ay, double bx, double by) {
        double dx = bx - ax, dy = by - ay;
        return StrictMath.sqrt(dx * dx + dy * dy);
    }

    // JS Math.round: floor(x + 0.5)
    private static long rnd(double x) {
        return (long) Math.floor(x + 0.5);
    }

    // ---- Bezier geometry + heading simulation ----

    // Samples the cubic (ex,ey,eh) -> (ax,ay,ah) with handles f1*d, f2*d into Work.S/D/E[slot]. Null when the piece leaves the
    // walls, has a cusp, or exceeds the curvature limit.
    static Geom geomOne(double ex, double ey, double eh, double ax, double ay, double ah, double f1, double f2,
                                Ctx c, int nSamples, int slot) {
        Work ws = c.ws;
        double dx = ax - ex, dy = ay - ey, d = StrictMath.sqrt(dx * dx + dy * dy);
        double x0 = ex, y0 = ey, x1 = ex + f1 * d * StrictMath.cos(eh), y1 = ey + f1 * d * StrictMath.sin(eh);
        double x2 = ax - f2 * d * StrictMath.cos(ah), y2 = ay - f2 * d * StrictMath.sin(ah), x3 = ax, y3 = ay;
        double len = 0, ek = 0, hmax = 0, tr = 0, px = x0, py = y0;
        double[] S = ws.S[slot], D = ws.D[slot], E = ws.E[slot], X = ws.X[slot], Y = ws.Y[slot];
        S[0] = 0;
        for (int i = 0; i <= nSamples; i++) {
            double t = (double) i / nSamples, u = 1 - t;
            double a0 = u * u * u, a1 = 3 * u * u * t, a2 = 3 * u * t * t, a3 = t * t * t;
            double bx = a0 * x0 + a1 * x1 + a2 * x2 + a3 * x3, by = a0 * y0 + a1 * y1 + a2 * y2 + a3 * y3;
            if (bx < c.lo[0] || bx > c.hi[0] || by < c.lo[1] || by > c.hi[1]) {
                ws.work += i + 1;
                return null;
            }
            double ddx = bx - px, ddy = by - py;
            len += StrictMath.sqrt(ddx * ddx + ddy * ddy);
            px = bx;
            py = by;
            S[i] = len;
            double vx = 3 * (u * u * (x1 - x0) + 2 * u * t * (x2 - x1) + t * t * (x3 - x2));
            double vy = 3 * (u * u * (y1 - y0) + 2 * u * t * (y2 - y1) + t * t * (y3 - y2));
            double sp = StrictMath.sqrt(vx * vx + vy * vy);
            if (sp < 0.12 * d) { // cusp / stall
                ws.work += i + 1;
                return null;
            }
            double ax2 = 6 * (u * (x2 - 2 * x1 + x0) + t * (x3 - 2 * x2 + x1));
            double ay2 = 6 * (u * (y2 - 2 * y1 + y0) + t * (y3 - 2 * y2 + y1));
            double k = (vx * ay2 - vy * ax2) / (sp * sp * sp), ak = Math.abs(k);
            if (ak > c.kmax) {
                ws.work += i + 1;
                return null;
            }
            ek += k * k * sp / nSamples;
            tr += ak * sp / nSamples * (i == 0 || i == nSamples ? 0.5 : 1); // total tangent rotation = integral |kappa| ds (trapezoid)
            if (ak > hmax) hmax = ak;
            D[i] = vx;
            E[i] = vy;
            X[i] = bx;
            Y[i] = by;
        }
        ws.work += nSamples + 1;
        return new Geom(f1, f2, len, nSamples, slot, tr, len + c.curvW * ek + PEAK_W * hmax, ah,
                x0, y0, x1, y1, x2, y2, x3, y3);
    }

    // Simulates the robot heading along g (speed profile, turn-rate cap) from heading h0 and speed v0 with speed cap vc and end
    // speed vJ. endT > 0: heading follows linear(h0, h1, endT) instead of the tangent. True when admissible; the final simulated
    // heading is left in ws.hEnd.
    static boolean simOk(Geom g, double h0, double endT, double v0, double vc, double endLag, Ctx c) {
        Work ws = c.ws;
        ws.work += g.n * 0.5;
        int N = g.n;
        double L = g.len, A = c.A, W = c.W, v1 = c.vJ, hEnd = g.h1;
        double[] S = ws.S[g.slot], D = ws.D[g.slot], E = ws.E[g.slot], X = ws.X[g.slot], Y = ws.Y[g.slot];
        double h = h0, prevS = 0, pv = v0, sl = c.bodySlack(X[0], Y[0], h0);
        if (Math.abs(wrap(StrictMath.atan2(E[0], D[0]) - h0)) > REV_LIM) return false;
        double dh = wrap(hEnd - h0), rc = StrictMath.sqrt(c.hl * c.hl + c.hw * c.hw);
        if (endT > 0 && Math.abs(dh) > 3.0) return false; // rotation direction ambiguous near 180 deg
        for (int i = 1; i <= N; i++) {
            double s = S[i];
            double va = StrictMath.sqrt(v0 * v0 + 2 * A * s), vd = StrictMath.sqrt(v1 * v1 + 2 * A * Math.max(0, L - s));
            double v = Math.min(Math.min(vc, va), vd);
            double dt = (s - prevS) / Math.max(1e-3, (v + pv) / 2);
            double s0 = prevS, hp = h, pv0 = pv;
            prevS = s;
            pv = v;
            double di = StrictMath.atan2(E[i], D[i]);
            double target = endT > 0 ? h0 + dh * Math.min(1, s / (endT * L)) : di;
            double e = wrap(target - h), m = W * dt;
            h += e > m ? m : e < -m ? -m : e;
            if (Math.abs(wrap(di - h)) > REV_LIM) return false;
            double sp = sl;
            sl = c.bodySlack(X[i], Y[i], h);
            if (sl < 0) return false; // whole body inside the bound at the simulated heading
            // between samples a body corner moves at most ds + rc*|dh|: the body can only leave the bound there when an end's slack is
            // below half of that; then the interval is re-checked on a finer grid
            double mv = s - s0 + rc * Math.abs(h - hp);
            if (2 * Math.min(sp, sl) < mv && !gapIn(c, g, (double) (i - 1) / N, (double) i / N, Math.ceil(mv / BOUND_STEP), s0, hp, pv0,
                    h0, dh, endT, L, v0, vc)) return false;
        }
        if (Math.abs(wrap(hEnd - h)) > endLag) return false;
        ws.hEnd = h;
        return true;
    }

    // Dense re-check of one sample interval of cubic q (segment naming x0..y1, cx1..cy2), parameter t0 -> t1 in k steps, starting at
    // arc length s0 with heading h and speed pv: the k-1 interior poses, heading simulated as in simOk, keep the whole body inside the bound.
    static boolean gapIn(Ctx c, Geom q, double t0, double t1, double k, double s0, double h, double pv, double h0, double dh,
                         double endT, double L, double v0, double vc) {
        c.ws.work += k * 1.5;
        double A = c.A, W = c.W, v1 = c.vJ;
        double s = s0, px = 0, py = 0;
        for (int j = 0; j < k; j++) {
            double t = t0 + (t1 - t0) * j / k, u = 1 - t, a0 = u * u * u, a1 = 3 * u * u * t, a2 = 3 * u * t * t, a3 = t * t * t;
            double bx = a0 * q.x0 + a1 * q.cx1 + a2 * q.cx2 + a3 * q.x1, by = a0 * q.y0 + a1 * q.cy1 + a2 * q.cy2 + a3 * q.y1;
            double ds = j == 0 ? 0 : StrictMath.sqrt((bx - px) * (bx - px) + (by - py) * (by - py));
            px = bx;
            py = by;
            if (j == 0) continue;
            s += ds;
            double v = Math.min(Math.min(vc, StrictMath.sqrt(v0 * v0 + 2 * A * s)), StrictMath.sqrt(v1 * v1 + 2 * A * Math.max(0, L - s)));
            double dt = ds / Math.max(1e-3, (v + pv) / 2);
            pv = v;
            double vx = u * u * (q.cx1 - q.x0) + 2 * u * t * (q.cx2 - q.cx1) + t * t * (q.x1 - q.cx2);
            double vy = u * u * (q.cy1 - q.y0) + 2 * u * t * (q.cy2 - q.cy1) + t * t * (q.y1 - q.cy2);
            double target = endT > 0 ? h0 + dh * Math.min(1, s / (endT * L)) : StrictMath.atan2(vy, vx);
            double e = wrap(target - h), m = W * dt;
            h += e > m ? m : e < -m ? -m : e;
            if (!c.bodyIn(bx, by, h)) return false;
        }
        return true;
    }

    // Cheap early collateral test of one cubic: weight of balls NOT in excl swept by the intake mouth (1) or the body (0.7).
    private static double collBez(Ctx c, double x0, double y0, double h0, double x1, double y1, double h1, double f1,
                                  double f2, boolean[] excl) {
        double[][] balls = c.balls;
        double d = dist(x0, y0, x1, y1);
        double X1 = x0 + f1 * d * StrictMath.cos(h0), Y1 = y0 + f1 * d * StrictMath.sin(h0);
        double X2 = x1 - f2 * d * StrictMath.cos(h1), Y2 = y1 - f2 * d * StrictMath.sin(h1);
        double mnx = Math.min(Math.min(x0, x1), Math.min(X1, X2)) - 14, mxx = Math.max(Math.max(x0, x1), Math.max(X1, X2)) + 14;
        double mny = Math.min(Math.min(y0, y1), Math.min(Y1, Y2)) - 14, mxy = Math.max(Math.max(y0, y1), Math.max(Y1, Y2)) + 14;
        int[] near = new int[balls.length];
        int nn = 0;
        for (int i = 0; i < balls.length; i++) {
            double[] b = balls[i];
            if (excl[i] || b[0] < mnx || b[0] > mxx || b[1] < mny || b[1] > mxy) continue;
            near[nn++] = i;
        }
        if (nn == 0) return 0;
        double reach = c.reach, halfW = c.p.intakeWidth / 2, bl = c.p.robotLength / 2 + 0.6, bw = c.p.robotWidth / 2 + 0.6;
        boolean[] done = new boolean[balls.length];
        double bad = 0;
        int n = (int) Math.max(3, Math.ceil(d * 1.4 / 0.8));
        c.ws.work += (n + 1) * (0.5 + 0.1 * nn);
        for (int k = 0; k <= n; k++) {
            double t = (double) k / n, u = 1 - t;
            double vx = 3 * (u * u * (X1 - x0) + 2 * u * t * (X2 - X1) + t * t * (x1 - X2));
            double vy = 3 * (u * u * (Y1 - y0) + 2 * u * t * (Y2 - Y1) + t * t * (y1 - Y2));
            double sp = StrictMath.sqrt(vx * vx + vy * vy);
            if (sp == 0 || Double.isNaN(sp)) sp = 1;
            double fx = vx / sp, fy = vy / sp;
            double px = u * u * u * x0 + 3 * u * u * t * X1 + 3 * u * t * t * X2 + t * t * t * x1;
            double py = u * u * u * y0 + 3 * u * u * t * Y1 + 3 * u * t * t * Y2 + t * t * t * y1;
            for (int q = 0; q < nn; q++) {
                int i = near[q];
                if (done[i]) continue;
                double dx = balls[i][0] - px, dy = balls[i][1] - py;
                double uu = dx * fx + dy * fy, vv = -dx * fy + dy * fx;
                if (uu >= reach - 1.5 && uu <= reach + 1.5 && vv >= -halfW && vv <= halfW) {
                    done[i] = true;
                    bad += 1;
                } else if (uu > -bl && uu < bl && vv > -bw && vv < bw) {
                    done[i] = true;
                    bad += 0.7;
                }
            }
        }
        return bad;
    }

    // collateral of a piece (single cubic or via pair); excl may be null (no collateral test)
    private static double pieceColl(Ctx c, double ex, double ey, double eh, double ax, double ay, double ah, double f1,
                                    double f2, Via via, boolean[] excl) {
        if (excl == null) return 0;
        if (via == null) return collBez(c, ex, ey, eh, ax, ay, ah, f1, f2, excl);
        return collBez(c, ex, ey, eh, via.x, via.y, via.h, f1, f2, excl)
                + collBez(c, via.x, via.y, via.h, ax, ay, ah, via.f1, via.f2, excl);
    }

    // ---- piece search ----

    // memo key: quantised end/start poses (+ start heading for the first piece) + ids of already-owned balls near the piece
    private static String pieceKey(Ctx c, double ex, double ey, double eh, double ax, double ay, double ah, double d,
                                   boolean first, double h0, boolean[] excl) {
        StringBuilder ek = new StringBuilder();
        if (excl != null) {
            double m = 1.3 * d + 14;
            double mnx = Math.min(ex, ax) - m, mxx = Math.max(ex, ax) + m, mny = Math.min(ey, ay) - m, mxy = Math.max(ey, ay) + m;
            for (int i = 0; i < c.balls.length; i++) {
                double[] b = c.balls[i];
                if (b[0] >= mnx && b[0] <= mxx && b[1] >= mny && b[1] <= mxy && excl[i]) ek.append(i).append(',');
            }
        }
        return ek + "|" + rnd(ex * 50) + "," + rnd(ey * 50) + "," + rnd(eh * 500) + "," + rnd(ax * 50) + "," + rnd(ay * 50) + ","
                + rnd(ah * 500) + "," + (first ? String.valueOf(rnd(h0 * 500)) : "x");
    }

    // Best admissible piece from pose (ex,ey,eh) to run-in pose (ax,ay,ah) for a robot at heading h0 / speed v0 at the start.
    // Tries single cubics first; the via stage only when c.via and none works.
    private static Piece evalPiece(double ex, double ey, double eh, double ax, double ay, double ah, Ctx c, double h0, double v0,
                                   boolean first, boolean[] excl) {
        double dx = ax - ex, dy = ay - ey, d = StrictMath.sqrt(dx * dx + dy * dy);
        if (d < 1e-6) return null;
        boolean pivot = first && Math.abs(wrap(eh - h0)) > PIVOT_TOL; // first tangent differs from the robot heading: heading linear from h0
        String key = pieceKey(c, ex, ey, eh, ax, ay, ah, d, first, h0, excl);
        if (c.memo.containsKey(key)) {
            Piece m = c.memo.get(key);
            if (m != null || !c.via || pivot) return m;
            return evalViaCached(key, ex, ey, eh, ax, ay, ah, c, h0, v0, excl);
        }
        Piece best = null;
        double brg = StrictMath.atan2(dy, dx);
        double need = c.p.pruneA * (Math.abs(wrap(brg - eh)) + Math.abs(wrap(ah - brg))) + c.p.pruneB;
        int passes = (first || d >= need) ? 2 : 0; // short hops with big turns cannot fit one cubic: straight to via
        for (int pass = 0; pass < passes; pass++) {
            if (pass == 1 && best != null) break; // long-handle loops only when the coarse handles found nothing
            double[][] hl = pass == 1 ? LOOPS : COARSE;
            for (int hi = 0; hi < hl.length; hi++) {
                double f1 = hl[hi][0], f2 = hl[hi][1];
                Geom g = geomOne(ex, ey, eh, ax, ay, ah, f1, f2, c, c.samples, 0);
                if (g == null) continue;
                for (int m = 0; m < MODE_END_T.length; m++) {
                    double endT = MODE_END_T[m], pen = MODE_PEN[m];
                    boolean slow = MODE_SLOW[m];
                    if (pivot && endT == 0) continue; // tangent heading would begin at the tangent, not at the robot heading
                    double lx = g.tr - Math.abs(wrap(ah - eh)); // loop excess: rotation beyond the turn the hop needs
                    double base = g.cost + pen + (slow ? g.len * 0.8 : 0) + c.p.loopW * Math.max(0, lx - c.p.loopRad);
                    if (best != null && base >= best.cost) continue;
                    if (!simOk(g, h0, endT, v0, slow ? c.vSlow : c.vFast, END_LAG, c)) continue;
                    double coll = pieceColl(c, ex, ey, eh, ax, ay, ah, f1, f2, null, excl);
                    double cost = base + c.p.pieceCollW * coll;
                    if (best != null && cost >= best.cost) continue;
                    best = new Piece(cost, f1, f2, g.len, endT, slow, null, coll, lx);
                    break;
                }
            }
        }
        c.memo.put(key, best);
        if (best == null && c.via && !pivot) return evalViaCached(key, ex, ey, eh, ax, ay, ah, c, h0, v0, excl);
        return best;
    }

    private static Piece evalViaCached(String key, double ex, double ey, double eh, double ax, double ay, double ah, Ctx c,
                                       double h0, double v0, boolean[] excl) {
        String vk = key + "v";
        if (c.memo.containsKey(vk)) return c.memo.get(vk);
        Piece best = viaPiece(ex, ey, eh, ax, ay, ah, c, h0, v0, excl);
        if (c.ws.work <= c.viaLimit) c.memo.put(vk, best); // a result cut short by the budget is not cached
        return best;
    }

    // Two-cubic "via" piece E -> V -> A (only when single cubics fail). V lies on an arc of radius r from the piece start, turned
    // by +-th; cubic 1 leaves along the start tangent and meets V tangentially, cubic 2 continues to the run-in pose; both use
    // tangent heading at approach speed and the same heading test (the end heading of cubic 1 seeds cubic 2).
    private static Piece viaPiece(double ex, double ey, double eh, double ax, double ay, double ah, Ctx c, double h0, double v0,
                                  boolean[] excl) {
        Piece r = viaPieceR(ex, ey, eh, ax, ay, ah, c, h0, v0, excl, c.p.viaR);
        if (r == null) r = viaPieceR(ex, ey, eh, ax, ay, ah, c, h0, v0, excl, c.p.viaR2);
        return r;
    }

    private static class ViaCand {
        final double vx, vy, vh, f, lb;

        ViaCand(double vx, double vy, double vh, double f, double lb) {
            this.vx = vx;
            this.vy = vy;
            this.vh = vh;
            this.f = f;
            this.lb = lb;
        }
    }

    private static Piece viaPieceR(double ex, double ey, double eh, double ax, double ay, double ah, Ctx c, double h0, double v0,
                                   boolean[] excl, double[] radii) {
        if (c.ws.work > c.viaLimit) return null;
        Params p = c.p;
        double dEA = dist(ex, ey, ax, ay);
        List<ViaCand> cand = new ArrayList<ViaCand>();
        for (int ri = 0; ri < radii.length; ri++) {
            for (int ti = 0; ti < p.viaTh.length; ti++) {
                for (int si = 0; si < 2; si++) {
                    double r = radii[ri], th0 = p.viaTh[ti], hv = eh + (si == 0 ? 1 : -1) * th0;
                    boolean pos = si == 0;
                    double vx = pos ? ex - r * StrictMath.sin(eh) + r * StrictMath.sin(hv)
                            : ex + r * StrictMath.sin(eh) - r * StrictMath.sin(hv);
                    double vy = pos ? ey + r * StrictMath.cos(eh) - r * StrictMath.cos(hv)
                            : ey - r * StrictMath.cos(eh) + r * StrictMath.cos(hv);
                    if (vx < c.lo[0] || vx > c.hi[0] || vy < c.lo[1] || vy > c.hi[1]) continue;
                    double dEV = dist(ex, ey, vx, vy);
                    // lb = lower bound of the piece length (arc + straight to the run-in); handle fraction of a circular arc
                    cand.add(new ViaCand(vx, vy, wrap(hv), (4.0 / 3) * StrictMath.tan(th0 / 4) * r / Math.max(1e-6, dEV),
                            r * th0 + dist(vx, vy, ax, ay)));
                }
            }
        }
        Collections.sort(cand, new Comparator<ViaCand>() { // stable: ties keep generation order
            @Override
            public int compare(ViaCand u, ViaCand z) {
                double d = u.lb - z.lb;
                return d < 0 ? -1 : d > 0 ? 1 : 0;
            }
        });
        Piece best = null;
        for (int qi = 0; qi < cand.size(); qi++) {
            ViaCand q = cand.get(qi);
            if (best != null && q.lb + 6 >= best.cost) break;
            if (q.lb > 3 * dEA + 50) continue;
            Geom g1 = geomOne(ex, ey, eh, q.vx, q.vy, q.vh, q.f, q.f, c, c.samples, 1);
            if (g1 == null) continue;
            if (best != null && g1.cost + p.viaSlowW * g1.len + p.viaPen >= best.cost) continue;
            if (!simOk(g1, h0, 0, v0, c.vSlow, p.viaLag, c)) continue;
            double hMid = c.ws.hEnd;
            for (int fi = 0; fi < VIA_F.length; fi++) {
                Geom g2 = geomOne(q.vx, q.vy, q.vh, ax, ay, ah, VIA_F[fi][0], VIA_F[fi][1], c, c.samples, 0);
                if (g2 == null) continue;
                double lx = g1.tr + g2.tr - Math.abs(wrap(ah - eh));
                double cost = g1.cost + g2.cost + p.viaPen + p.viaSlowW * (g1.len + g2.len) + p.loopW * Math.max(0, lx - p.loopRad);
                if (best != null && cost >= best.cost) continue;
                if (!simOk(g2, hMid, 0, c.vSlow, c.vSlow, p.viaLag, c)) continue;
                Via via = new Via(q.vx, q.vy, q.vh, VIA_F[fi][0], VIA_F[fi][1]);
                double coll = pieceColl(c, ex, ey, eh, ax, ay, ah, q.f, q.f, via, excl);
                double total = cost + p.pieceCollW * coll;
                if (best != null && total >= best.cost) continue;
                best = new Piece(total, q.f, q.f, g1.len + g2.len, 0, true, via, coll, lx);
            }
        }
        return best;
    }

    // Dense curvature check of one cubic: true when max |kappa| over n+1 samples stays within kmax * kTol. The coarse search
    // samples can miss a sharp bend on a long chord, so every route and refined piece is re-checked with this.
    private static boolean curvOk(Ctx c, double x0, double y0, double h0, double x1, double y1, double h1, double f1, double f2) {
        int n = c.p.kCheckN;
        double d = dist(x0, y0, x1, y1), lim = c.kmax * c.p.kTol;
        double ax1 = x0 + f1 * d * StrictMath.cos(h0), ay1 = y0 + f1 * d * StrictMath.sin(h0);
        double ax2 = x1 - f2 * d * StrictMath.cos(h1), ay2 = y1 - f2 * d * StrictMath.sin(h1);
        c.ws.work += n * 0.5;
        for (int i = 0; i <= n; i++) {
            double t = (double) i / n, u = 1 - t;
            double vx = 3 * (u * u * (ax1 - x0) + 2 * u * t * (ax2 - ax1) + t * t * (x1 - ax2));
            double vy = 3 * (u * u * (ay1 - y0) + 2 * u * t * (ay2 - ay1) + t * t * (y1 - ay2));
            double qx = 6 * (u * (ax2 - 2 * ax1 + x0) + t * (x1 - 2 * ax2 + ax1));
            double qy = 6 * (u * (ay2 - 2 * ay1 + y0) + t * (y1 - 2 * ay2 + ay1));
            double sp2 = vx * vx + vy * vy;
            if (Math.abs(vx * qy - vy * qx) > lim * sp2 * StrictMath.sqrt(sp2)) return false;
        }
        return true;
    }

    // dense curvature check of every piece of a DP route (S = robot start)
    private static boolean routeCurvOk(Ctx c, List<State> items, Pose3 S) {
        for (int i = 0; i < items.size(); i++) {
            State s = items.get(i);
            Piece pc = s.piece;
            if (pc == null || s.merge || s.direct) continue;
            double x0 = i == 0 ? S.x : items.get(i - 1).ex, y0 = i == 0 ? S.y : items.get(i - 1).ey;
            double h0 = i == 0 ? s.hs : items.get(i - 1).h;
            if (pc.via != null) {
                Via v = pc.via;
                if (!curvOk(c, x0, y0, h0, v.x, v.y, v.h, pc.f1, pc.f2) || !curvOk(c, v.x, v.y, v.h, s.ax, s.ay, s.h, v.f1, v.f2))
                    return false;
            } else if (!curvOk(c, x0, y0, h0, s.ax, s.ay, s.h, pc.f1, pc.f2)) return false;
        }
        return true;
    }

    // final polish of a chosen single cubic: finer grid, local search on the handle fractions; via pieces are kept as found
    private static Piece refinePiece(double ex, double ey, double eh, double ax, double ay, double ah, Ctx c, Piece cur,
                                     double h0, double v0, boolean[] excl, boolean first) {
        if (cur.via != null) return cur;
        Piece best0 = refineTry(ex, ey, eh, ax, ay, ah, c, cur.f1, cur.f2, cur.slow, h0, v0, excl, first);
        Piece best = best0 != null ? best0 : cur;
        for (int pass = 0; pass < 2; pass++) {
            Piece b0 = best;
            double st = pass == 1 ? 0.04 : 0.12;
            for (int ia = -1; ia <= 1; ia++) {
                for (int ib = -1; ib <= 1; ib++) {
                    if (ia == 0 && ib == 0) continue;
                    double f1 = b0.f1 + ia * st, f2 = b0.f2 + ib * st;
                    if (f1 < 0.08 || f2 < 0.08) continue;
                    Piece r = refineTry(ex, ey, eh, ax, ay, ah, c, f1, f2, cur.slow, h0, v0, excl, first);
                    if (r != null && r.cost < best.cost) best = r;
                }
            }
        }
        return best;
    }

    private static Piece refineTry(double ex, double ey, double eh, double ax, double ay, double ah, Ctx c, double f1, double f2,
                                   boolean curSlow, double h0, double v0, boolean[] excl, boolean first) {
        Geom g = geomOne(ex, ey, eh, ax, ay, ah, f1, f2, c, 24, 0);
        if (g == null) return null;
        boolean pivot = first && Math.abs(wrap(eh - h0)) > PIVOT_TOL;
        for (int m = 0; m < MODE_END_T.length; m++) {
            double endT = MODE_END_T[m], pen = MODE_PEN[m];
            boolean slow = MODE_SLOW[m];
            if (curSlow && !slow) continue; // never trade a slow piece for a fast one
            if (pivot && endT == 0) continue;
            if (simOk(g, h0, endT, v0, slow ? c.vSlow : c.vFast, END_LAG, c)) {
                if (!curvOk(c, ex, ey, eh, ax, ay, ah, f1, f2)) return null;
                double coll = pieceColl(c, ex, ey, eh, ax, ay, ah, f1, f2, null, excl);
                double lx = g.tr - Math.abs(wrap(ah - eh));
                return new Piece(g.cost + pen + (slow ? g.len * 0.8 : 0) + c.p.loopW * Math.max(0, lx - c.p.loopRad)
                        + c.p.pieceCollW * coll, f1, f2, g.len, endT, slow, null, coll, lx);
            }
        }
        return null;
    }

    // ---- groups (single balls and pairs) ----

    static List<Group> makeGroups(double[][] balls, Params p) {
        List<Group> g = new ArrayList<Group>();
        for (int i = 0; i < balls.length; i++) g.add(new Group(i, -1, balls[i][0], balls[i][1], 0, 0));
        if (p.capacity >= 2) {
            for (int i = 0; i < balls.length; i++) {
                for (int j = i + 1; j < balls.length; j++) {
                    if (dist(balls[i][0], balls[i][1], balls[j][0], balls[j][1]) <= p.intakeWidth - p.ballDiameter + 1e-9) {
                        double perp = wrap(StrictMath.atan2(balls[j][1] - balls[i][1], balls[j][0] - balls[i][0]) + Math.PI / 2);
                        g.add(new Group(i, j, (balls[i][0] + balls[j][0]) / 2, (balls[i][1] + balls[j][1]) / 2, perp,
                                wrap(perp + Math.PI)));
                    }
                }
            }
        }
        return g;
    }

    // ---- top-M sequence search (DFS, branch and bound) ----
    // hop cost = straight distance + stopC + turnPenalty * |heading change| + corridorW per unused ball in the hop's corridor

    private static class TopState {
        final Params p;
        final double[][] balls;
        final Group[] groups;
        final int M;
        final List<Seq> out = new ArrayList<Seq>();
        double cap = Double.POSITIVE_INFINITY, stopC, work = 0;
        final boolean[] used;
        final Group[] cur;
        final int[][] order;
        final double[][] dist;
        final int[][] corr;

        TopState(Params p, double[][] balls, Group[] groups, int k) {
            this.p = p;
            this.balls = balls;
            this.groups = groups;
            M = p.topM;
            used = new boolean[balls.length];
            cur = new Group[k];
            int m = groups.length;
            order = new int[m + 1][];
            dist = new double[m + 1][];
            corr = new int[(m + 1) * m][];
            stopC = (1 / Math.max(0.05, p.approachSpeed)) * (p.runIn + p.driveThrough);
        }

        double bound() {
            return Math.min(cap, out.size() >= M ? out.get(M - 1).cost : Double.POSITIVE_INFINITY);
        }
    }

    private static double tsHeading(Group g, double px, double py, double ph, double d) {
        if (d < 1e-9) return ph;
        double bear = StrictMath.atan2(g.y - py, g.x - px);
        if (g.b < 0) return bear;
        return Math.abs(wrap(g.perp - bear)) <= Math.abs(wrap(g.alt - bear)) ? g.perp : g.alt;
    }

    // groups ordered by distance from a point (ties by index), cached per origin
    private static void tsSortFrom(TopState ts, int o, double px, double py) {
        Group[] groups = ts.groups;
        int m = groups.length;
        final double[] d2 = new double[m];
        Integer[] idx = new Integer[m];
        for (int i = 0; i < m; i++) {
            double dx = groups[i].x - px, dy = groups[i].y - py;
            d2[i] = dx * dx + dy * dy;
            idx[i] = i;
        }
        Arrays.sort(idx, new Comparator<Integer>() {
            @Override
            public int compare(Integer a, Integer z) {
                double d = d2[a] - d2[z];
                if (d != 0) return d < 0 ? -1 : 1;
                return a - z;
            }
        });
        int[] ord = new int[m];
        double[] dd = new double[m];
        for (int i = 0; i < m; i++) {
            ord[i] = idx[i];
            dd[i] = StrictMath.sqrt(d2[idx[i]]);
        }
        ts.order[o] = ord;
        ts.dist[o] = dd;
        ts.work += m * 3;
    }

    // unused balls lying in the corridor swept by the straight hop (px,py) -> group g (memoised per origin/target)
    private static int[] tsCorridor(TopState ts, int o, double px, double py, Group g, int gi, boolean fromStart) {
        int idx = o * ts.groups.length + gi;
        int[] r = ts.corr[idx];
        if (r != null) return r;
        int[] tmp = new int[ts.balls.length];
        int cnt = 0;
        double dx = g.x - px, dy = g.y - py, dl = StrictMath.sqrt(dx * dx + dy * dy);
        if (dl > 1e-6) {
            double ux = dx / dl, uy = dy / dl, a0 = fromStart ? 3 : 1.5, ch = ts.p.corridorHalf;
            double[][] b = ts.balls;
            for (int i = 0; i < b.length; i++) {
                if (i == g.a || i == g.b) continue;
                double rx = b[i][0] - px, ry = b[i][1] - py, al = rx * ux + ry * uy, lt = -rx * uy + ry * ux;
                if (al > a0 && al < dl - 1.5 && lt > -ch && lt < ch) tmp[cnt++] = i;
            }
        }
        r = Arrays.copyOf(tmp, cnt);
        ts.corr[idx] = r;
        return r;
    }

    private static void tsDfs(TopState ts, int o, double px, double py, double ph, double cst, int left, int depth) {
        ts.work += 1;
        if (left == 0) {
            if (cst < ts.bound()) {
                Seq e = new Seq(cst, Arrays.copyOf(ts.cur, depth));
                List<Seq> out = ts.out;
                int i = out.size();
                out.add(e);
                while (i > 0 && out.get(i - 1).cost > cst) {
                    out.set(i, out.get(i - 1));
                    i--;
                }
                out.set(i, e);
                if (out.size() > ts.M) out.remove(out.size() - 1);
                ts.cap = Math.min(ts.cap, out.get(0).cost * 1.6 + 25);
            }
            return;
        }
        if (ts.order[o] == null) tsSortFrom(ts, o, px, py);
        int[] od = ts.order[o];
        double[] dd = ts.dist[o];
        Group[] groups = ts.groups;
        boolean[] used = ts.used;
        for (int q = 0; q < od.length; q++) {
            double dgap = dd[q];
            if (cst + dgap + ts.stopC >= ts.bound()) break;
            Group g = groups[od[q]];
            if (g.size > left || used[g.a] || (g.b >= 0 && used[g.b])) continue;
            double h = tsHeading(g, px, py, ph, dgap);
            double nc = cst + dgap + ts.stopC + ts.p.turnPenalty * Math.abs(wrap(h - ph));
            if (nc >= ts.bound()) continue;
            int[] cl = tsCorridor(ts, o, px, py, g, od[q], depth == 0);
            for (int z = 0; z < cl.length; z++) if (!used[cl[z]]) nc += ts.p.corridorW;
            if (nc >= ts.bound()) continue;
            ts.cur[depth] = g;
            used[g.a] = true;
            if (g.b >= 0) used[g.b] = true;
            tsDfs(ts, od[q], g.x, g.y, h, nc, left - g.size, depth + 1);
            used[g.a] = false;
            if (g.b >= 0) used[g.b] = false;
        }
    }

    // up to topM sequences sorted by cost; adds its (small, fixed) work to ws
    private static List<Seq> topSeqs(double sx, double sy, double sh, double[][] balls, int k, Params p, List<Group> groups,
                                     Work ws) {
        TopState ts = new TopState(p, balls, groups.toArray(new Group[0]), k);
        tsDfs(ts, ts.groups.length, sx, sy, sh, 0, k, 0); // origin index m = the robot start
        ws.work += ts.work;
        return ts.out;
    }

    // ---- DP over heading candidates for one sequence ----

    private static State mkState(Ctx c, Group g, double h, double o, State chainOf, double L) {
        double ch = StrictMath.cos(h), sh = StrictMath.sin(h);
        double s = 0;
        if (g.size == 2) {
            double rx = c.balls[g.b][0] - c.balls[g.a][0], ry = c.balls[g.b][1] - c.balls[g.a][1];
            double ul = Math.abs(-rx * sh + ry * ch) / 2, lim = o == 0 ? (c.p.intakeWidth - c.p.ballDiameter) / 2 + 1e-9 : c.p.lateral;
            if (Math.abs(o) + ul > lim) return null; // one of the two balls would miss the intake
            s = Math.abs(rx * ch + ry * sh) / 2;
        }
        double kx = g.x - c.reach * ch + o * sh, ky = g.y - c.reach * sh - o * ch; // ball at +o on the left normal of the path
        State st = new State(g, h, o, chainOf, kx, ky, L, s, kx - (L + s) * ch, ky - (L + s) * sh, kx + (c.d + s) * ch,
                ky + (c.d + s) * sh);
        // straight run-in / drive-through at the constant heading h: both ends (and the contact pose) with the whole body inside the bound
        if (!c.bodyIn(st.ax, st.ay, h) || !c.bodyIn(st.kx, st.ky, h) || !c.bodyIn(st.ex, st.ey, h)) return null;
        return st;
    }

    private static List<Double> dedupeAngles(double[] list, int n, double tol) {
        List<Double> out = new ArrayList<Double>();
        for (int i = 0; i < n; i++) {
            boolean dup = false;
            for (int j = 0; j < out.size(); j++) {
                if (Math.abs(wrap(out.get(j) - list[i])) < tol) {
                    dup = true;
                    break;
                }
            }
            if (!dup) out.add(list[i]);
        }
        return out;
    }

    // offsets {0, +lateral, -lateral} at which single g's straight-on pose at heading h fits the bound (mkState: no work units)
    private static List<Double> fitOffs(Ctx c, Group g, double h, State chain) {
        List<Double> r = new ArrayList<Double>();
        double[] offs = c.p.lateral > 0 ? new double[] {0, c.p.lateral, -c.p.lateral} : new double[] {0};
        for (int oi = 0; oi < offs.length; oi++) if (mkState(c, g, h, offs[oi], chain, c.L) != null) r.add(offs[oi]);
        return r;
    }

    // arrival heading h: as is when it fits at o = 0; near the bound (tier 0) off-centre, or else the feasible pose nearest to it.
    // Tier 0 only: at tier 1 the extra states cost halfBounds-03 / lines-10 / presetBounds-10 their loop-free route.
    private static void pushArrival(Ctx c, Group g, double h, State chain, List<Cand> cands) {
        if (c.tier == 0 && g.size == 1 && mkState(c, g, h, 0, chain, c.L) == null) {
            List<Double> fo = fitOffs(c, g, h, chain);
            for (int q = 0; q < fo.size(); q++) cands.add(new Cand(h, fo.get(q), chain, 0));
            if (fo.isEmpty()) {
                double[] f = nearFeas(g, h);
                if (f != null) cands.add(new Cand(f[0], f[1], chain, 0));
            }
        } else cands.add(new Cand(h, 0, chain, 0));
    }

    private static List<Double> candHeadings(Ctx c, Group[] seq, int i, Pose3 S) {
        Group g = seq[i];
        double pvx = i == 0 ? S.x : seq[i - 1].x, pvy = i == 0 ? S.y : seq[i - 1].y;
        double din = dirOf(pvx, pvy, g.x, g.y);
        double[] out = new double[18];
        int n = 0;
        if (g.size == 2) { // perpendicular first; the others only if both balls fit (mkState)
            out[n++] = g.perp;
            out[n++] = g.alt;
        }
        out[n++] = din;
        if (i + 1 < seq.length) {
            Group nx = seq[i + 1];
            double dout = dirOf(g.x, g.y, nx.x, nx.y), dcat = dirOf(pvx, pvy, nx.x, nx.y);
            out[n++] = dout;
            out[n++] = dcat;
            out[n++] = wrap(dcat + 0.4);
            out[n++] = wrap(dcat - 0.4);
            out[n++] = wrap(din + wrap(dout - din) / 2);
        } else {
            out[n++] = wrap(din + 0.3);
            out[n++] = wrap(din - 0.3);
        }
        if (i > 0 && i + 1 >= seq.length) { // U-turn: back along the previous stop's line
            double ppx = i == 1 ? S.x : seq[i - 2].x, ppy = i == 1 ? S.y : seq[i - 2].y;
            out[n++] = wrap(dirOf(ppx, ppy, pvx, pvy) + Math.PI);
        }
        if (g.size != 2 && c.tier >= 2) for (int q = 0; q < 8; q++) out[n++] = wrap(q * Math.PI / 4);
        return dedupeAngles(out, n, 0.07);
    }

    private static String groupKey(Group g) {
        return g.a + "." + g.b;
    }

    // seq: groups; S: robot start; bound: best total cost so far (Infinity = none yet).
    // Returns the route or null; learns dead prefixes / bad hops when it fails with no bound.
    private static Dp runDP(Ctx c, Group[] seq, Pose3 S, double bound) {
        Params p = c.p;
        int nb = c.balls.length;
        List<State> prev = null;
        for (int i = 0; i < seq.length; i++) {
            Group g = seq[i];
            List<State> states = new ArrayList<State>();
            boolean[] excl = new boolean[nb]; // balls owned by stops 0..i
            for (int q = 0; q <= i; q++) {
                excl[seq[q].a] = true;
                if (seq[q].b >= 0) excl[seq[q].b] = true;
            }
            StringBuilder pk = new StringBuilder();
            for (int q = 0; q <= i; q++) pk.append(q > 0 ? "," : "").append(groupKey(seq[q]));
            String pkey = pk.toString();
            if (c.dead.contains(pkey)) return null; // identical prefix already proven infeasible
            List<Double> base = candHeadings(c, seq, i, S);
            List<Cand> cands = new ArrayList<Cand>();
            double[] offs = g.size == 1 && p.lateral > 0 ? new double[] {0, p.lateral, -p.lateral} : new double[] {0};
            for (int hi = 0; hi < base.size(); hi++) for (int oi = 0; oi < offs.length; oi++) cands.add(new Cand(base.get(hi), offs[oi], null, 0));
            // joint line with the next single: the steepest headings off the line ball->next ball at which both balls still sit inside the
            // intake strip (opposite sides, o = -sep/2), so the next stop chains onto this straight instead of swinging round to line up.
            // Tier 0 only: at tier 1 the extra states spent halfBounds-43's budget before its usual route (pass-1 success lost).
            Group nx = i + 1 < seq.length ? seq[i + 1] : null;
            if (c.tier == 0 && g.size == 1 && nx != null && nx.size == 1) {
                double D = dist(g.x, g.y, nx.x, nx.y), q = 2 * 0.99 * p.lateral / D;
                if (q < 1) {
                    double phi = dirOf(g.x, g.y, nx.x, nx.y), del = StrictMath.asin(q);
                    for (int sg = -1; sg <= 1; sg += 2) {
                        double h = wrap(phi + sg * del);
                        cands.add(new Cand(h, -D * StrictMath.sin(phi - h) / 2, null, 0));
                    }
                }
            }
            if (i > 0 && g.size == 1) {
                for (int pi = 0; pi < prev.size(); pi++) {
                    State pst = prev.get(pi);
                    double ch = StrictMath.cos(pst.h), sh = StrictMath.sin(pst.h);
                    double rx = g.x - pst.kx, ry = g.y - pst.ky;
                    double w = -rx * sh + ry * ch, along = rx * ch + ry * sh - c.reach;
                    if (Math.abs(w) <= p.chainTol && along - pst.s >= 3) cands.add(new Cand(pst.h, w, pst, 0)); // share the previous line
                }
            }
            // arrival headings: the stop heading follows the path (circular arc from the previous drive-through end, or from the start, into
            // the ball) instead of forcing a swing to line up; at the first stop also straight on along the robot heading, ball off-centre
            if (i == 0) {
                double w = -(g.x - S.x) * StrictMath.sin(S.h) + (g.y - S.y) * StrictMath.cos(S.h);
                if (Math.abs(w) <= p.lateral) cands.add(new Cand(S.h, w, null, 0));
                pushArrival(c, g, wrap(2 * dirOf(S.x, S.y, g.x, g.y) - S.h), null, cands);
            } else {
                for (int pi = 0; pi < prev.size(); pi++) {
                    State pst = prev.get(pi);
                    pushArrival(c, g, wrap(2 * dirOf(pst.ex, pst.ey, g.x, g.y) - pst.h), pst, cands);
                }
            }
            for (int ci = 0; ci < cands.size(); ci++) {
                for (int li = 0; li < c.Lopts.length; li++) {
                    Cand cd = cands.get(ci);
                    if (c.ws.work > c.ws.limit) return null; // stage budget exhausted mid-DP: abandon (nothing is learned from it)
                    State st = mkState(c, g, cd.h, cd.o, cd.chain, c.Lopts[li]);
                    if (st == null) continue;
                    double slowC = c.slowF * (st.L + c.d + 2 * st.s) + p.shortW * (c.L - st.L);
                    if (i == 0) dpFirst(c, st, S, excl, slowC);
                    else dpNext(c, st, cd.chain, prev, excl, slowC, bound);
                    if (st.cost < Double.POSITIVE_INFINITY && st.cost < bound + 15) states.add(st);
                }
            }
            if (states.isEmpty()) {
                if (bound == Double.POSITIVE_INFINITY) {
                    c.dead.add(pkey);
                    c.badHop.add((i > 0 ? groupKey(seq[i - 1]) : "S") + ">" + groupKey(g));
                }
                return null;
            }
            Collections.sort(states, new Comparator<State>() { // stable
                @Override
                public int compare(State u, State v) {
                    double d = u.cost - v.cost;
                    return d < 0 ? -1 : d > 0 ? 1 : 0;
                }
            });
            while (states.size() > p.beam) states.remove(states.size() - 1);
            prev = states;
        }
        State best = prev.get(0);
        for (int i = 1; i < prev.size(); i++) if (prev.get(i).cost < best.cost) best = prev.get(i);
        List<State> items = new ArrayList<State>();
        for (State s = best; s != null; s = s.prev) items.add(s);
        Collections.reverse(items);
        return new Dp(best.cost, items);
    }

    // first stop: initial tangent hs within startTanMax of the robot heading (robot heading, bearing to the run-in, +-0.35 around the
    // bearing, the stop heading, S.h +-0.6, S.h +-startTanMax; clamped); the heading itself always starts at the robot heading
    private static void dpFirst(Ctx c, State st, Pose3 S, boolean[] excl, double slowC) {
        Params p = c.p;
        double m = p.startTanMax;
        double br = dirOf(S.x, S.y, st.ax, st.ay);
        double[] raw = {S.h, limTan(S.h, m, br), limTan(S.h, m, br + 0.35), limTan(S.h, m, br - 0.35), limTan(S.h, m, st.h),
                wrap(S.h + 0.6), wrap(S.h - 0.6), wrap(S.h + m), wrap(S.h - m)};
        List<Double> hsl = dedupeAngles(raw, raw.length, 0.05);
        // direct start: the robot already sits on the stop's line with enough straight left -> it drives straight on while its
        // heading rotates linear(S.h, st.h, endT) (a collinear cubic when the heading has to change), converged >= 4 in before the ball
        double ch = StrictMath.cos(st.h), sh2 = StrictMath.sin(st.h), rx = st.kx - S.x, ry = st.ky - S.y;
        double along = rx * ch + ry * sh2, lat = -rx * sh2 + ry * ch;
        if (st.s == 0 && Math.abs(lat) < 0.3 && along >= 1.2 && along < st.L + 0.01 && along - p.directStraight >= 0) {
            double dEndT = -1;
            if (Math.abs(wrap(st.h - S.h)) < DIRECT_TOL) dEndT = 0;
            else {
                double full = (along - p.directStraight) / (along + c.d);
                for (int fi = 0; fi < DIRECT_FR.length && dEndT < 0; fi++) {
                    double t = full * DIRECT_FR[fi];
                    if (t > 0.01 && directOk(c, S, st, along, t)) dEndT = t;
                }
            }
            if (dEndT >= 0) {
                double tot = p.startTurnW * Math.abs(wrap(st.h - S.h)) + c.slowF * (along + c.d)
                        + p.shortW * Math.max(0, p.runIn - along);
                if (tot < st.cost) {
                    st.cost = tot;
                    st.piece = null;
                    st.hs = st.h;
                    st.direct = true;
                    st.dEndT = dEndT;
                }
            }
        }
        for (int q = 0; q < hsl.size(); q++) {
            double hs = hsl.get(q);
            Piece pc = evalPiece(S.x, S.y, hs, st.ax, st.ay, st.h, c, S.h, 0, true, excl);
            if (pc == null) continue;
            double tot = pc.cost + p.startTurnW * Math.abs(wrap(hs - S.h)) + slowC;
            if (tot < st.cost) {
                st.cost = tot;
                st.piece = pc;
                st.hs = hs;
                st.direct = false;
                st.dEndT = 0;
            }
        }
    }

    // clamp angle a to within m of the robot heading h
    private static double limTan(double h, double m, double a) {
        double d = wrap(a - h);
        return d > m ? wrap(h + m) : d < -m ? wrap(h - m) : a;
    }

    // heading simulation of the direct start (straight line of length along + d from rest at approach speed): linear(h0, h1, endT) with
    // the turn-rate cap; ok when the heading never leaves REV_LIM of the line and has converged (0.05 rad) from 4 in before the ball
    private static boolean directOk(Ctx c, Pose3 S, State st, double along, double endT) {
        double h0 = S.h, h1 = st.h;
        int N = 24;
        double L = along + c.d, A = c.A, W = c.W, dh = wrap(h1 - h0), v1 = c.vJ, sChk = along - 4.0;
        c.ws.work += N * 0.5;
        double h = h0, ps = 0, pv = 0;
        for (int i = 1; i <= N; i++) {
            double s = L * i / N;
            double v = Math.min(Math.min(c.vSlow, StrictMath.sqrt(2 * A * s)), StrictMath.sqrt(v1 * v1 + 2 * A * Math.max(0, L - s)));
            double dt = (s - ps) / Math.max(1e-3, (v + pv) / 2);
            ps = s;
            pv = v;
            double e = wrap(h0 + dh * Math.min(1, s / (endT * L)) - h), mx = W * dt;
            h += e > mx ? mx : e < -mx ? -mx : e;
            double err = Math.abs(wrap(h1 - h));
            if (err > REV_LIM || (s >= sChk && err > 0.05)) return false;
            if (!c.bodyIn(S.x + (st.ex - S.x) * i / N, S.y + (st.ey - S.y) * i / N, h)) return false;
        }
        return true;
    }

    // later stop: best predecessor state (exact collinear merge, or a piece from the predecessor's drive-through end)
    private static void dpNext(Ctx c, State st, State chain, List<State> prev, boolean[] excl, double slowC, double bound) {
        for (int pi = 0; pi < prev.size(); pi++) {
            State pst = prev.get(pi);
            if (chain != null && chain != pst) continue;
            double tot = Double.POSITIVE_INFINITY;
            Piece pc = null;
            boolean merge = false;
            if (st.s == 0 && pst.s == 0 && Math.abs(wrap(st.h - pst.h)) < 0.02) { // same heading and same line: just drive on (no piece)
                double ch = StrictMath.cos(pst.h), sh = StrictMath.sin(pst.h);
                double rx = st.kx - pst.kx, ry = st.ky - pst.ky;
                double lat = -rx * sh + ry * ch, gap = rx * ch + ry * sh;
                if (Math.abs(lat) < 0.05 && gap >= 3) {
                    merge = true;
                    pc = null;
                    tot = pst.cost + c.slowF * (gap - st.L - c.d) + slowC;
                }
            }
            if (!merge) {
                double lb = pst.cost + slowC + dist(pst.ex, pst.ey, st.ax, st.ay);
                if (lb >= st.cost || lb >= bound + 15) continue;
                pc = evalPiece(pst.ex, pst.ey, pst.h, st.ax, st.ay, st.h, c, pst.h, c.vJ, false, excl);
                if (pc == null) continue;
                tot = pst.cost + pc.cost + slowC;
            }
            if (tot < st.cost) {
                st.cost = tot;
                st.prev = pst;
                st.piece = pc;
                st.merge = merge;
            }
        }
    }

    // ---- collateral of a whole route ----
    // Walks the route (straights and cubics) at 0.8 in steps and counts balls the intake mouth sweeps that are not planned (1
    // each), planned balls swept early (earlyW: on a curve, or before their own run-in) or clipped by the body (clipW), and
    // unplanned balls touched by the body (0.25).

    private static class Col {
        final double[][] balls;
        final Work ws;
        final double reach, halfW, bl, bw, earlyW, clipW;
        final boolean[] done, chosen;
        final int[] owner;
        double bad = 0;
        int curItem = 0;
        boolean inCurve = false;
        final List<State> items;
        final List<Double> px = new ArrayList<Double>(), py = new ArrayList<Double>(); // path trace (points >= 1 in apart) for selfCross

        Col(double[][] balls, Work ws, double reach, Params p, List<State> items) {
            this.balls = balls;
            this.ws = ws;
            this.reach = reach;
            halfW = p.intakeWidth / 2;
            bl = p.robotLength / 2 + 0.6;
            bw = p.robotWidth / 2 + 0.6;
            earlyW = p.earlyW;
            clipW = p.clipW;
            done = new boolean[balls.length];
            chosen = new boolean[balls.length];
            owner = new int[balls.length];
            Arrays.fill(owner, -1);
            this.items = items;
        }
    }

    private static void colTest(Col cs, double x, double y, double h) {
        double[][] balls = cs.balls;
        double fx = StrictMath.cos(h), fy = StrictMath.sin(h);
        cs.ws.work += 0.1 + 0.03 * balls.length;
        int np = cs.px.size();
        if (np == 0 || dist(cs.px.get(np - 1), cs.py.get(np - 1), x, y) >= 1) { // path trace for selfCross
            cs.px.add(x);
            cs.py.add(y);
        }
        for (int i = 0; i < balls.length; i++) {
            if (cs.done[i]) continue;
            double dx = balls[i][0] - x, dy = balls[i][1] - y;
            if (dx > 14 || dx < -14 || dy > 14 || dy < -14) continue;
            double u = dx * fx + dy * fy, v = -dx * fy + dy * fx;
            if (u >= cs.reach - 1.5 && u <= cs.reach + 1.5 && v >= -cs.halfW && v <= cs.halfW) {
                cs.done[i] = true;
                if (!cs.chosen[i]) cs.bad += 1;
                else if (cs.inCurve || !colOwner(cs, i)) cs.bad += cs.earlyW;
            } else if (u > -cs.bl && u < cs.bl && v > -cs.bw && v < cs.bw) {
                cs.done[i] = true;
                cs.bad += cs.chosen[i] ? cs.clipW : 0.25;
            }
        }
    }

    // a planned ball swept on item curItem's straight is fine if it belongs to it (or to merged followers)
    private static boolean colOwner(Col cs, int i) {
        int o = cs.owner[i];
        if (o == cs.curItem) return true;
        if (o < 0 || o < cs.curItem) return false;
        for (int j = cs.curItem + 1; j <= o; j++) if (!cs.items.get(j).merge) return false;
        return true;
    }

    private static void colLine(Col cs, double x0, double y0, double x1, double y1, double h) {
        int n = (int) Math.max(1, Math.ceil(dist(x0, y0, x1, y1) / 0.8));
        for (int k = 0; k <= n; k++) colTest(cs, x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n, h);
    }

    private static void colBez(Col cs, double x0, double y0, double h0, double x1, double y1, double h1, double f1, double f2) {
        double d = dist(x0, y0, x1, y1);
        double X1 = x0 + f1 * d * StrictMath.cos(h0), Y1 = y0 + f1 * d * StrictMath.sin(h0);
        double X2 = x1 - f2 * d * StrictMath.cos(h1), Y2 = y1 - f2 * d * StrictMath.sin(h1);
        int n = (int) Math.max(2, Math.ceil(d * 1.4 / 0.8));
        for (int k = 0; k <= n; k++) {
            double t = (double) k / n, u = 1 - t;
            double vx = 3 * (u * u * (X1 - x0) + 2 * u * t * (X2 - X1) + t * t * (x1 - X2));
            double vy = 3 * (u * u * (Y1 - y0) + 2 * u * t * (Y2 - Y1) + t * t * (y1 - Y2));
            colTest(cs, u * u * u * x0 + 3 * u * u * t * X1 + 3 * u * t * t * X2 + t * t * t * x1,
                    u * u * u * y0 + 3 * u * u * t * Y1 + 3 * u * t * t * Y2 + t * t * t * y1, StrictMath.atan2(vy, vx));
        }
    }

    // the (single or via) cubic(s) leading to item s's run-in
    private static void colPiece(Col cs, double x0, double y0, double h0, State s) {
        Piece pc = s.piece;
        Via v = pc != null ? pc.via : null;
        cs.inCurve = true;
        if (v != null) {
            colBez(cs, x0, y0, h0, v.x, v.y, v.h, pc.f1, pc.f2);
            colBez(cs, v.x, v.y, v.h, s.ax, s.ay, s.h, v.f1, v.f2);
        } else colBez(cs, x0, y0, h0, s.ax, s.ay, s.h, pc != null ? pc.f1 : 0.4, pc != null ? pc.f2 : 0.4);
        cs.inCurve = false;
    }

    private static double collateral(List<State> items, Pose3 S, Ctx c, Group[] seq) {
        Col cs = new Col(c.balls, c.ws, c.reach, c.p, items);
        for (int j = 0; j < items.size(); j++) {
            cs.owner[items.get(j).g.a] = j;
            if (items.get(j).g.b >= 0) cs.owner[items.get(j).g.b] = j;
        }
        for (int q = 0; q < seq.length; q++) {
            cs.chosen[seq[q].a] = true;
            if (seq[q].b >= 0) cs.chosen[seq[q].b] = true;
        }
        for (int i = 0; i < items.size(); i++) {
            State s = items.get(i), q = i > 0 ? items.get(i - 1) : null;
            cs.curItem = i;
            if (i == 0) {
                if (s.direct) {
                    colLine(cs, S.x, S.y, s.ex, s.ey, s.h);
                    continue;
                }
                colPiece(cs, S.x, S.y, s.hs, s);
                colLine(cs, s.ax, s.ay, s.ex, s.ey, s.h);
            } else if (s.merge) colLine(cs, q.ex, q.ey, s.ex, s.ey, s.h);
            else {
                colPiece(cs, q.ex, q.ey, q.h, s);
                colLine(cs, s.ax, s.ay, s.ex, s.ey, s.h);
            }
        }
        return cs.bad + c.p.crossB * selfCross(cs);
    }

    // Number of times the traced route (points >= 1 in apart) crosses itself; neighbouring chords are not compared. A crossing means the
    // route doubles back over itself (a loop), which the driver reads as swinging around instead of driving directly.
    private static int selfCross(Col cs) {
        int n = cs.px.size();
        double[] X = new double[n], Y = new double[n];
        for (int i = 0; i < n; i++) {
            X[i] = cs.px.get(i);
            Y[i] = cs.py.get(i);
        }
        int cnt = 0;
        cs.ws.work += (double) n * n * 0.02;
        for (int i = 0; i + 1 < n; i++) {
            for (int j = i + 3; j + 1 < n; j++) {
                double d1 = (X[i + 1] - X[i]) * (Y[j] - Y[i]) - (Y[i + 1] - Y[i]) * (X[j] - X[i]);
                double d2 = (X[i + 1] - X[i]) * (Y[j + 1] - Y[i]) - (Y[i + 1] - Y[i]) * (X[j + 1] - X[i]);
                if (d1 * d2 >= 0) continue;
                double d3 = (X[j + 1] - X[j]) * (Y[i] - Y[j]) - (Y[j + 1] - Y[j]) * (X[i] - X[j]);
                double d4 = (X[j + 1] - X[j]) * (Y[i + 1] - Y[j]) - (Y[j + 1] - Y[j]) * (X[i + 1] - X[j]);
                if (d3 * d4 < 0) cnt++;
            }
        }
        return cnt;
    }

    // ---- search stage ----
    // One pass over the candidate sequences with context c (tier, via on/off). bound0 = cost of the best route kept from earlier
    // stages (Infinity if none): DP runs are pruned against it, and only strictly better routes are returned. The stage stops
    // when it has spent `cap` work units.
    private static Dp searchStage(Ctx c, List<Seq> cands, Pose3 S, double sx, double sy, double sh, double cap, double bound0,
                                  boolean haveKept, Work ws, Params p, boolean oneSet) {
        int tier = c.tier;
        Map<String, Integer> subsetCount = new HashMap<String, Integer>();
        List<Seq> deferred = new ArrayList<Seq>(), over = new ArrayList<Seq>();
        Dp best = null;
        int feasible = 0, tried = 0;
        boolean stop = false;
        double w0 = ws.work;
        int maxTry = c.via ? p.viaTries : tier == 0 ? p.maxDp : tier == 2 ? p.maxTier2 : p.maxDp;
        // pass 2: when every sequence takes the same balls (k == reachable balls) the per-subset cap alone would end the search after
        // perSubset orderings; the orderings it skipped are tried while the stage has no route at all
        for (int pass = 0; pass < 3 && !stop && (pass < 2 || (oneSet && best == null)); pass++) {
            List<Seq> list = pass == 0 ? cands : pass == 1 ? deferred : over;
            for (int li = 0; li < list.size(); li++) {
                Seq cd = list.get(li);
                // learned pruning: a hop already proven infeasible makes every sequence using it a long shot -> try those last
                if (pass == 0 && !c.badHop.isEmpty()) {
                    boolean bad = false;
                    String pk = "S";
                    for (int q = 0; q < cd.seq.length && !bad; q++) {
                        String gk = groupKey(cd.seq[q]);
                        if (c.badHop.contains(pk + ">" + gk)) bad = true;
                        pk = gk;
                    }
                    if (bad) {
                        deferred.add(cd);
                        continue;
                    }
                }
                List<String> ids = new ArrayList<String>();
                for (int q = 0; q < cd.seq.length; q++) ids.add(cd.seq[q].a + (cd.seq[q].b >= 0 ? "+" + cd.seq[q].b : ""));
                Collections.sort(ids);
                StringBuilder kb = new StringBuilder();
                for (int q = 0; q < ids.size(); q++) kb.append(q > 0 ? "," : "").append(ids.get(q));
                String key = kb.toString();
                int cnt = subsetCount.containsKey(key) ? subsetCount.get(key) : 0;
                if (pass < 2 && cnt >= p.perSubset) {
                    over.add(cd);
                    continue;
                }
                subsetCount.put(key, cnt + 1);
                if (++tried > maxTry) {
                    stop = true;
                    break;
                }
                if (haveKept && ws.work - ws.base > p.hardWork) {
                    stop = true;
                    break;
                }
                if (ws.work - w0 > cap) {
                    stop = true;
                    break;
                }
                S.x = sx;
                S.y = sy;
                S.h = sh;
                Dp r = runDP(c, cd.seq, S, best != null ? best.cost : bound0);
                if (r == null || !routeCurvOk(c, r.items, S)) continue;
                feasible++;
                r.coll = collateral(r.items, S, c, cd.seq);
                r.cost += p.collateralW * r.coll;
                for (int q = 0; q < r.items.size(); q++) {
                    Piece pc = r.items.get(q).piece;
                    if (pc != null && !r.items.get(q).merge && pc.lx > p.loopRad) r.loops++;
                }
                if (best == null || r.cost < best.cost) best = r;
                if (feasible >= (tier == 0 ? p.wantFeasible : p.wantFeasible2) && best.coll == 0 && best.loops == 0) {
                    stop = true;
                    break;
                }
            }
        }
        return best;
    }

    // ---- plan ----

    private static Plan emptyPlan(int tier, double work, String reason) {
        return new Plan(new ArrayList<Stop>(), 0, new ArrayList<RouteItem>(), tier, work, reason);
    }

    private static final int REACH_HEADS = 36;

    // Bound reachability: a group can be collected only if, for some straight-on pose set (run-in start, contact, drive-through end at one
    // heading), the whole body stays inside the bound. Singles: 36 headings x lateral offsets {0, +lateral, -lateral}; pairs: perp / alt.
    // Run-in lengths are the ones the tiers use. Costs no work units (fixed small loop).
    static boolean groupReachable(Group g, Params p, Bound bd, double hl, double hw) {
        double[] hs;
        if (g.size == 2) hs = new double[] {g.perp, g.alt};
        else {
            hs = new double[REACH_HEADS];
            for (int q = 0; q < REACH_HEADS; q++) hs[q] = wrap(q * 2 * Math.PI / REACH_HEADS);
        }
        double[] offs = g.size == 1 && p.lateral > 0 ? new double[] {0, p.lateral, -p.lateral} : new double[] {0};
        double[] Ls = {p.runIn, 5.3, 3.5};
        boolean ok = false;
        g.feas = new ArrayList<double[]>(); // [heading, offset] poses straight-on feasible at the full run-in (pushArrival fallback near the bound)
        for (int hi = 0; hi < hs.length; hi++) {
            double h = hs[hi], ch = StrictMath.cos(h), sh = StrictMath.sin(h);
            for (int oi = 0; oi < offs.length; oi++) {
                double o = offs[oi], kx = g.x - p.intakeReach * ch + o * sh, ky = g.y - p.intakeReach * sh - o * ch;
                double ex = kx + p.driveThrough * ch, ey = ky + p.driveThrough * sh;
                if (!bodyIn(bd, hl, hw, kx, ky, h) || !bodyIn(bd, hl, hw, ex, ey, h)) continue;
                for (int li = 0; li < Ls.length; li++) {
                    if (bodyIn(bd, hl, hw, kx - Ls[li] * ch, ky - Ls[li] * sh, h)) {
                        ok = true;
                        if (li == 0) g.feas.add(new double[] {h, o});
                        break;
                    }
                }
            }
        }
        return ok;
    }

    // feasible pose of g (from groupReachable) whose heading is nearest to t, or null
    private static double[] nearFeas(Group g, double t) {
        double[] best = null;
        double bd = Double.POSITIVE_INFINITY;
        for (int q = 0; q < g.feas.size(); q++) {
            double d = Math.abs(wrap(g.feas.get(q)[0] - t));
            if (d < bd) {
                bd = d;
                best = g.feas.get(q);
            }
        }
        return best;
    }

    public static Plan plan(double sx, double sy, double sh, List<double[]> ballList, Params p) {
        double[][] balls = ballList.toArray(new double[0][]);
        int n = balls.length;
        if (Math.min(p.capacity, n) <= 0) return emptyPlan(0, 0, "no balls");
        Bound bd = sanitizeBound(p);
        double hl = p.robotLength / 2, hw = p.robotWidth / 2;
        if (!bodyIn(bd, hl, hw, sx, sy, sh)) return emptyPlan(0, 0, "start outside bound");
        List<Group> groups = new ArrayList<Group>();
        for (Group g : makeGroups(balls, p)) if (groupReachable(g, p, bd, hl, hw)) groups.add(g);
        boolean[] reach = new boolean[n];
        for (Group g : groups) {
            reach[g.a] = true;
            if (g.b >= 0) reach[g.b] = true;
        }
        int nReach = 0;
        for (int i = 0; i < n; i++) if (reach[i]) nReach++;
        int kMax = Math.min(p.capacity, nReach);
        if (kMax <= 0) return emptyPlan(0, 0, "no reachable balls");
        Work ws = new Work();
        Pose3 S = new Pose3(sx, sy, sh);
        // Fewer-balls fallback: if no continuous route exists for k balls, try k-1 ... 1. Every attempt has its own work budget counted
        // from its own start (ws.base); the plan's work is the total over all attempts.
        for (int k = kMax; k >= 1; k--) {
            ws.base = ws.work;
            Object[] keep = planK(k, sx, sy, sh, balls, p, groups, ws, S, bd, k == nReach);
            if (keep != null) return finish((Dp) keep[0], S, (Ctx) keep[1], ws.work);
        }
        return emptyPlan(-1, ws.work, "no continuous route"); // nothing feasible for even 1 ball: empty plan, no driving
    }

    private static Object[] planK(int k, double sx, double sy, double sh, double[][] balls, Params p, List<Group> groups, Work ws,
            Pose3 S, Bound bd, boolean oneSet) {
        List<Seq> cands = topSeqs(sx, sy, sh, balls, k, p, groups, ws); // identical for every stage: once
        List<Map<String, Piece>> memos = new ArrayList<Map<String, Piece>>(); // per tier, shared by its plain and via stages
        for (int i = 0; i < 3; i++) memos.add(new HashMap<String, Piece>());
        Dp keepBest = null;
        Ctx keepCtx = null;
        List<double[]> stages = new ArrayList<double[]>(Arrays.asList(p.stages));
        stages.addAll(Arrays.asList(p.rescue)); // rescue stages only run while no route exists at all
        for (int si = 0; si < stages.size(); si++) {
            // clean route found (no collateral, no looping piece, no self-crossing), or out of budget
            if (keepBest != null && ((keepBest.coll == 0 && keepBest.loops == 0) || ws.work - ws.base > p.hardWork)) break;
            double[] st = stages.get(si);
            boolean rescue = si >= p.stages.length;
            if (rescue && keepBest != null) break;
            int tier = (int) st[0];
            boolean via = st[1] == 1;
            double cap = st[2];
            Ctx c = new Ctx(p, tier, ws, balls, memos.get(tier), via, bd);
            ws.limit = ws.work + cap * p.dpOvershoot; // a DP in progress may overshoot the stage cap by this factor
            c.viaLimit = ws.work + cap; // via results cut short by the stage budget are not memoised
            Dp best = searchStage(c, cands, S, sx, sy, sh, cap, keepBest != null ? keepBest.cost : Double.POSITIVE_INFINITY,
                    keepBest != null, ws, p, oneSet);
            if (best != null && (keepBest == null || best.cost < keepBest.cost)) {
                keepBest = best;
                keepCtx = c;
            }
        }
        return keepBest == null ? null : new Object[] { keepBest, keepCtx };
    }

    private static Plan finish(Dp best, Pose3 S, Ctx c, double work) {
        int nb = c.balls.length;
        for (int i = 0; i < best.items.size(); i++) {
            State s = best.items.get(i);
            if (s.piece == null || s.merge || s.direct) continue;
            State q = i > 0 ? best.items.get(i - 1) : null;
            boolean[] excl = new boolean[nb];
            for (int z = 0; z <= i; z++) {
                excl[best.items.get(z).g.a] = true;
                if (best.items.get(z).g.b >= 0) excl[best.items.get(z).g.b] = true;
            }
            s.piece = i == 0 ? refinePiece(S.x, S.y, s.hs, s.ax, s.ay, s.h, c, s.piece, S.h, 0, excl, true)
                    : refinePiece(q.ex, q.ey, q.h, s.ax, s.ay, s.h, c, s.piece, q.h, c.vJ, excl, false);
        }
        List<Stop> stops = new ArrayList<Stop>();
        List<RouteItem> route = new ArrayList<RouteItem>();
        for (State s : best.items) {
            stops.add(new Stop(s.g.x, s.g.y, s.h, s.g.size == 2 ? new int[] {s.g.a, s.g.b} : new int[] {s.g.a}));
            route.add(new RouteItem(s));
        }
        return new Plan(stops, best.cost, route, c.tier, work, "");
    }

    // ---- segments ----

    private static Segment curveSeg(double x0, double y0, double th0, double x1, double y1, double h1, double f1, double f2,
                                    int stop, double hh0, double endT, boolean slow) {
        double d = dist(x0, y0, x1, y1), h0 = endT > 0 ? hh0 : th0;
        return new Segment(Mode.CURVE, x0, y0, x1, y1, h0, h1, slow, stop, x0 + f1 * d * StrictMath.cos(th0),
                y0 + f1 * d * StrictMath.sin(th0), x1 - f2 * d * StrictMath.cos(h1), y1 - f2 * d * StrictMath.sin(h1),
                endT > 0 ? endT : 0);
    }

    private static Segment lineSeg(double x0, double y0, double x1, double y1, double h, int stop) {
        return new Segment(Mode.CONSTANT, x0, y0, x1, y1, h, h, true, stop, 0, 0, 0, 0, 0);
    }

    // segments for the piece leading to run-in (s.ax, s.ay): one cubic, or two for a via piece. (x0,y0,th0) = start pose /
    // tangent, hh0 = robot heading at the start (used only by linear heading)
    private static void pieceSegs(List<Segment> segs, double x0, double y0, double th0, double hh0, RouteItem s, int stop) {
        if (s.via != null) {
            Via v = s.via;
            segs.add(curveSeg(x0, y0, th0, v.x, v.y, v.h, s.f1, s.f2, stop, hh0, 0, s.pslow));
            segs.add(curveSeg(v.x, v.y, v.h, s.ax, s.ay, s.h, v.f1, v.f2, stop, v.h, 0, s.pslow));
        } else if (dist(x0, y0, s.ax, s.ay) > 1e-3) {
            segs.add(curveSeg(x0, y0, th0, s.ax, s.ay, s.h, s.f1, s.f2, stop, hh0, s.endT, s.pslow));
        }
    }

    public static List<Segment> buildSegments(Plan plan, double startX, double startY, double startHeading, Params p) {
        List<Segment> segs = new ArrayList<Segment>();
        List<RouteItem> it = plan.route;
        for (int i = 0; i < it.size(); i++) {
            RouteItem s = it.get(i), q = i > 0 ? it.get(i - 1) : null;
            if (i == 0) {
                if (s.direct) {
                    if (s.dEndT > 0) { // collinear cubic: heading linear(robot heading, stop heading, dEndT)
                        double tx = s.ex - startX, ty = s.ey - startY;
                        segs.add(new Segment(Mode.CURVE, startX, startY, s.ex, s.ey, startHeading, s.h, true, 0, startX + tx / 3,
                                startY + ty / 3, startX + 2 * tx / 3, startY + 2 * ty / 3, s.dEndT));
                    } else segs.add(lineSeg(startX, startY, s.ex, s.ey, s.h, 0));
                    continue;
                }
                pieceSegs(segs, startX, startY, s.hs, startHeading, s, 0);
            } else if (s.merge) {
                segs.add(lineSeg(q.ex, q.ey, s.ex, s.ey, s.h, i));
                continue;
            } else pieceSegs(segs, q.ex, q.ey, q.h, q.h, s, i);
            segs.add(lineSeg(s.ax, s.ay, s.ex, s.ey, s.h, i));
        }
        return segs;
    }

    public static String describe(Plan plan, List<double[]> balls) {
        StringBuilder sb = new StringBuilder("Balls:");
        for (int i = 0; i < balls.size(); i++)
            sb.append(String.format(Locale.US, " %d(%.1f,%.1f)", i, balls.get(i)[0], balls.get(i)[1]));
        sb.append("\nPicking: ").append(plan.chosen.length).append(" of ").append(balls.size());
        StringBuilder pairs = new StringBuilder(), order = new StringBuilder();
        for (Stop s : plan.stops) {
            String id = s.pair() ? s.balls[0] + "+" + s.balls[1] : "" + s.balls[0];
            if (s.pair()) pairs.append(pairs.length() > 0 ? ", " : "").append(id);
            order.append(order.length() > 0 ? " -> " : "").append(id);
        }
        sb.append("\nPairs: ").append(pairs.length() > 0 ? pairs : "none");
        sb.append("\nOrder: ").append(order);
        sb.append(String.format(Locale.US, "\nCost: %.1f", plan.cost));
        if (plan.empty() && !balls.isEmpty()) sb.append("\nNo continuous route: not driving (" + plan.reason + ")");
        return sb.toString();
    }
}
