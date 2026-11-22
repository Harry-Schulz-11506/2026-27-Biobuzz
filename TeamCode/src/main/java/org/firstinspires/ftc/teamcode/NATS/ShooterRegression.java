package org.firstinspires.ftc.teamcode.NATS;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.List;

/**
 * Field position -> flywheel RPM, built to be tuned live in FTC Dashboard (open "ShooterRegression").
 *
 * Every shot is one row of FOUR numbers: X, Y, R (the RPM that scored) and V (battery volts, 0 = not recorded).
 * No distance is stored. The table is EMPTY: it is a placeholder until you tune it.
 *
 * ONE GOAL, ALL FOUR: you tune the table at ONE goal only, the reference goal (REF_GOAL_X / REF_GOAL_Y,
 * default RED CELL 2). For any of the 4 goals the robot's position is first moved into that goal's frame:
 *     effective = REF_GOAL + (robot - selected goal)         (X is flipped for BLUE when MIRROR_FOR_BLUE is on)
 * so the same spot relative to the goal gets the same RPM, whichever goal you shoot at. The X, Y in the table
 * rows are these effective positions. L3 records them for you, so you can tune from any goal you like.
 *
 * HOW TO TUNE
 *  1. Shoot from a spot. If it was short/long, edit that row's R in Dashboard. It applies instantly.
 *  2. Need a new spot? Fill the next empty row (X, Y and R). A row with R = 0 is unused.
 *     Or press L3 on controller 2 in TeleOp: it records the robot's X, Y, the RPM and the battery voltage.
 *  3. Everything still off by the same amount? Change SHOT_OFFSET_RPM (adds to every shot).
 *  4. Pick how the rows become a surface with FIT_TYPE (below). Start with 1.
 *  5. Dashboard changes are lost on restart: copy the final values into the fields below.
 *
 * FIT_TYPE
 *  0 = least-squares fit through all the rows: a flat plane (degree 1) or a curved surface (degree 2, needs 6+ rows).
 *      degree = REGRESSION_DEGREE from MainTeleOp/AutoActions (0 = one average RPM).
 *  1 = inverse-distance blend: every row is hit EXACTLY, and between rows the RPM is a smooth mix of the nearby
 *      rows (closer rows count more, IDW_POWER sets how much more). Far from every row it drifts toward the
 *      average, so shoot from a few spots across the area you really use.
 * The result is always clamped to MIN/MAX RPM.
 */
@Config
public class ShooterRegression {

    // Built-in rows (none yet: the table below is empty, a placeholder to be filled by tuning).
    // Format when you add some: effective X, Y (see REF_GOAL below), RPM, volts.
    public static final double[] DEFAULT_X = {};
    public static final double[] DEFAULT_Y = {};
    public static final double[] DEFAULT_RPM = {};
    public static final double[] DEFAULT_VOLT = {};

    // ---- Voltage ----
    // The flywheel controller already scales its power for the battery, so the table should hold at any voltage.
    // If your data shows shots still change with voltage, set VOLT_RPM_PER_VOLT (RPM added per volt above VOLT_REF).
    public static double VOLT_RPM_PER_VOLT = 0;  // 0 = off
    public static double VOLT_REF = 12.5;        // battery voltage the table is assumed to be for

    // ---- Reference goal: the goal the table is tuned at. All 4 goals are converted into its frame. ----
    public static double REF_GOAL_X = 56.0;      // RED CELL 2 (default). TODO: change if you tune at another goal
    public static double REF_GOAL_Y = 52.0;
    public static boolean MIRROR_FOR_BLUE = true; // BLUE goals: flip X so "left of the goal" stays "left of the goal" from the blue side

    // ---- Live-tunable options ----
    public static int FIT_TYPE = 1;              // see above
    public static double IDW_POWER = 2.0;        // FIT_TYPE 1: higher = the nearest row dominates more (2 is a good start)
    public static double SHOT_OFFSET_RPM = 0;    // added to every prediction

    // ---- Live-tunable shot table (X, Y = robot position in inches, R = RPM, V = battery volts when recorded;
    // R = 0 means the row is empty, V = 0 means the voltage was not recorded) ----
    public static double X1 = 0, Y1 = 0, R1 = 0, V1 = 0;
    public static double X2 = 0, Y2 = 0, R2 = 0, V2 = 0;
    public static double X3 = 0, Y3 = 0, R3 = 0, V3 = 0;
    public static double X4 = 0, Y4 = 0, R4 = 0, V4 = 0;
    public static double X5 = 0, Y5 = 0, R5 = 0, V5 = 0;
    public static double X6 = 0, Y6 = 0, R6 = 0, V6 = 0;
    public static double X7 = 0, Y7 = 0, R7 = 0, V7 = 0;
    public static double X8 = 0, Y8 = 0, R8 = 0, V8 = 0;
    public static double X9 = 0, Y9 = 0, R9 = 0, V9 = 0;
    public static double X10 = 0, Y10 = 0, R10 = 0, V10 = 0;
    public static double X11 = 0, Y11 = 0, R11 = 0, V11 = 0;
    public static double X12 = 0, Y12 = 0, R12 = 0, V12 = 0;
    public static double X13 = 0, Y13 = 0, R13 = 0, V13 = 0;
    public static double X14 = 0, Y14 = 0, R14 = 0, V14 = 0;
    public static double X15 = 0, Y15 = 0, R15 = 0, V15 = 0;
    public static double X16 = 0, Y16 = 0, R16 = 0, V16 = 0;

    private static final int SLOTS = 16;
    private static final double MID = 72.0; // field centre: coordinates are shifted to it before fitting

    private int requestedDegree;
    private int degreeUsed = 0;
    private double rSquared = 1.0;
    private double[] coeffs = new double[6]; // FIT_TYPE 0: c0 + c1*u + c2*v + c3*u^2 + c4*u*v + c5*v^2, u = (x-72)/72, v = (y-72)/72
    private List<double[]> pts = new ArrayList<>(); // each {x, y, rpm, volts}

    public ShooterRegression(int degree) {
        refit(degree);
    }

    // ---------- table access ----------

    private static double[] slot(int i) {
        switch (i) {
            case 0: return new double[]{X1, Y1, R1, V1};
            case 1: return new double[]{X2, Y2, R2, V2};
            case 2: return new double[]{X3, Y3, R3, V3};
            case 3: return new double[]{X4, Y4, R4, V4};
            case 4: return new double[]{X5, Y5, R5, V5};
            case 5: return new double[]{X6, Y6, R6, V6};
            case 6: return new double[]{X7, Y7, R7, V7};
            case 7: return new double[]{X8, Y8, R8, V8};
            case 8: return new double[]{X9, Y9, R9, V9};
            case 9: return new double[]{X10, Y10, R10, V10};
            case 10: return new double[]{X11, Y11, R11, V11};
            case 11: return new double[]{X12, Y12, R12, V12};
            case 12: return new double[]{X13, Y13, R13, V13};
            case 13: return new double[]{X14, Y14, R14, V14};
            case 14: return new double[]{X15, Y15, R15, V15};
            default: return new double[]{X16, Y16, R16, V16};
        }
    }

    private static void setSlot(int i, double x, double y, double r, double v) {
        switch (i) {
            case 0: X1 = x; Y1 = y; R1 = r; V1 = v; break;
            case 1: X2 = x; Y2 = y; R2 = r; V2 = v; break;
            case 2: X3 = x; Y3 = y; R3 = r; V3 = v; break;
            case 3: X4 = x; Y4 = y; R4 = r; V4 = v; break;
            case 4: X5 = x; Y5 = y; R5 = r; V5 = v; break;
            case 5: X6 = x; Y6 = y; R6 = r; V6 = v; break;
            case 6: X7 = x; Y7 = y; R7 = r; V7 = v; break;
            case 7: X8 = x; Y8 = y; R8 = r; V8 = v; break;
            case 8: X9 = x; Y9 = y; R9 = r; V9 = v; break;
            case 9: X10 = x; Y10 = y; R10 = r; V10 = v; break;
            case 10: X11 = x; Y11 = y; R11 = r; V11 = v; break;
            case 11: X12 = x; Y12 = y; R12 = r; V12 = v; break;
            case 12: X13 = x; Y13 = y; R13 = r; V13 = v; break;
            case 13: X14 = x; Y14 = y; R14 = r; V14 = v; break;
            case 14: X15 = x; Y15 = y; R15 = r; V15 = v; break;
            default: X16 = x; Y16 = y; R16 = r; V16 = v; break;
        }
    }

    /** Rows currently in the table (empty rows skipped), each {x, y, rpm, volts}. */
    private static List<double[]> readTable() {
        List<double[]> list = new ArrayList<>();
        for (int i = 0; i < SLOTS; i++) {
            double[] s = slot(i);
            if (s[2] > 0) list.add(s);
        }
        return list;
    }

    // ---------- API used by MainTeleOp / AutoActions ----------

    public boolean usingRecorded() { return recordedCount() > 0; }
    public int recordedCount() { return Math.max(0, readTable().size() - DEFAULT_RPM.length); }
    public int getRequestedDegree() { return requestedDegree; }
    public int getDegreeUsed() { return degreeUsed; }
    public double getRSquared() { return rSquared; }
    public double[] getCoefficients() { return coeffs.clone(); }

    /** Robot X moved into the reference goal's frame (see the class comment). */
    public static double referenceX(double x, double goalX, boolean blue) {
        double dx = x - goalX;
        return REF_GOAL_X + ((blue && MIRROR_FOR_BLUE) ? -dx : dx);
    }

    /** Robot Y moved into the reference goal's frame. */
    public static double referenceY(double y, double goalY) {
        return REF_GOAL_Y + (y - goalY);
    }

    /** Adds a shot taken at the selected goal (robot position, goal position, RPM, battery volts or 0). */
    public void addSample(double x, double y, double goalX, double goalY, boolean blue, double rpm, double volts) {
        addReferenceSample(referenceX(x, goalX, blue), referenceY(y, goalY), rpm, volts);
    }

    /** Adds a row that is already in the reference frame: the first empty row, or the row nearest to it if all are used. */
    public void addReferenceSample(double x, double y, double rpm, double volts) {
        for (int i = 0; i < SLOTS; i++) {
            if (slot(i)[2] <= 0) { setSlot(i, x, y, rpm, volts); refit(requestedDegree); return; }
        }
        int nearest = 0;
        double best = Double.MAX_VALUE;
        for (int i = 0; i < SLOTS; i++) {
            double[] s = slot(i);
            double d = Math.hypot(s[0] - x, s[1] - y);
            if (d < best) { best = d; nearest = i; }
        }
        setSlot(nearest, x, y, rpm, volts);
        refit(requestedDegree);
    }

    /** Puts the table back to the built-in measured shots and clears the offset. */
    public void clearRecorded() {
        for (int i = 0; i < SLOTS; i++) {
            if (i < DEFAULT_RPM.length) setSlot(i, DEFAULT_X[i], DEFAULT_Y[i], DEFAULT_RPM[i], DEFAULT_VOLT[i]);
            else setSlot(i, 0, 0, 0, 0);
        }
        SHOT_OFFSET_RPM = 0;
        refit(requestedDegree);
    }

    /** The table as "(x,y):rpm" entries, for copying into the defaults. */
    public String describeSamples() {
        StringBuilder sb = new StringBuilder();
        for (double[] s : readTable()) {
            if (sb.length() > 0) sb.append(", ");
            sb.append(String.format("(%.1f,%.1f):%.0f", s[0], s[1], s[2]));
            if (s[3] > 0) sb.append(String.format("@%.1fV", s[3]));
        }
        return sb.toString();
    }

    /**
     * What the rows recorded with a battery voltage say about voltage: fits rpm = a + b*x + c*y + d*(volts - VOLT_REF)
     * through those rows and returns d (RPM per volt), or NaN if there are too few rows or the voltage barely varied.
     * Estimate only: it is never applied. Copy a trusted value into VOLT_RPM_PER_VOLT yourself.
     */
    public double getVoltageEffectRpmPerVolt() {
        List<double[]> v = new ArrayList<>();
        for (double[] s : pts) if (s[3] > 0) v.add(s);
        if (v.size() < 5) return Double.NaN;
        double lo = Double.POSITIVE_INFINITY, hi = Double.NEGATIVE_INFINITY;
        for (double[] s : v) { lo = Math.min(lo, s[3]); hi = Math.max(hi, s[3]); }
        if (hi - lo < 0.3) return Double.NaN;
        int m = 4;
        double[][] a = new double[m][m + 1];
        for (double[] s : v) {
            double[] b = {1, (s[0] - MID) / MID, (s[1] - MID) / MID, s[3] - VOLT_REF};
            for (int i = 0; i < m; i++) {
                for (int j = 0; j < m; j++) a[i][j] += b[i] * b[j];
                a[i][m] += b[i] * s[2];
            }
        }
        double[] sol = solve(a, m, 1e-9);
        return sol == null ? Double.NaN : sol[3];
    }

    /** One line for telemetry: the voltage estimate, and how far to trust it. */
    public String describeVoltageEffect() {
        int n = 0;
        double lo = Double.POSITIVE_INFINITY, hi = Double.NEGATIVE_INFINITY;
        for (double[] s : pts) if (s[3] > 0) { n++; lo = Math.min(lo, s[3]); hi = Math.max(hi, s[3]); }
        double e = getVoltageEffectRpmPerVolt();
        if (Double.isNaN(e)) return String.format("not enough voltage data yet (%d rows with volts, need 5+ over 0.3 V+)", n);
        String trust = (n < 10 || hi - lo < 1.0) ? "weak: too few rows or too narrow a voltage range" : "ok";
        return String.format("%+.0f RPM/V from %d rows (%.1f-%.1f V), %s", e, n, lo, hi, trust);
    }

    /** How far the current surface is from each row, e.g. "(40.4,5.7):+12". Use this to see which row to nudge. */
    public String describeErrors() {
        StringBuilder sb = new StringBuilder();
        for (double[] s : pts) {
            if (sb.length() > 0) sb.append(", ");
            sb.append(String.format("(%.1f,%.1f):%+.0f", s[0], s[1], evaluate(s[0], s[1]) - s[2]));
        }
        return sb.toString();
    }

    /**
     * Predicted RPM for a robot at (x, y) shooting at the goal (goalX, goalY), plus SHOT_OFFSET_RPM, clamped to
     * [minRpm, maxRpm]. Works for all 4 goals (blue = the BLUE alliance). Adds VOLT_RPM_PER_VOLT * (volts - VOLT_REF)
     * when volts is known (> 0) and that setting is not 0. An empty table gives minRpm.
     */
    public double predict(double x, double y, double goalX, double goalY, boolean blue,
                          double volts, double minRpm, double maxRpm) {
        refit(requestedDegree); // picks up Dashboard edits immediately (at most 16 rows, very cheap)
        double r = evaluate(referenceX(x, goalX, blue), referenceY(y, goalY)) + SHOT_OFFSET_RPM;
        if (volts > 0) r += VOLT_RPM_PER_VOLT * (volts - VOLT_REF);
        return Math.max(minRpm, Math.min(maxRpm, r));
    }

    /** Re-read the table and re-run the fit. degree only matters for FIT_TYPE 0. */
    public void refit(int degree) {
        requestedDegree = Math.max(0, Math.min(2, degree));
        pts = readTable();
        coeffs = new double[6];
        degreeUsed = 0;
        rSquared = 1.0;
        if (pts.isEmpty()) return;

        if (FIT_TYPE == 1) {
            degreeUsed = 1;
            return; // exact through every row
        }
        fitPoly(requestedDegree);
        rSquared = computeR2();
    }

    // ---------- evaluation ----------

    private double evaluate(double x, double y) {
        if (pts.isEmpty()) return 0;
        if (pts.size() == 1) return pts.get(0)[2];
        if (FIT_TYPE == 1) return idw(x, y);
        double u = (x - MID) / MID, v = (y - MID) / MID;
        return coeffs[0] + coeffs[1] * u + coeffs[2] * v + coeffs[3] * u * u + coeffs[4] * u * v + coeffs[5] * v * v;
    }

    /** Inverse-distance weighting: exact at every row, a smooth blend of the nearby rows in between. */
    private double idw(double x, double y) {
        double num = 0, den = 0;
        for (double[] p : pts) {
            double d = Math.hypot(x - p[0], y - p[1]);
            if (d < 1e-6) return p[2];
            double w = 1.0 / Math.pow(d, Math.max(0.1, IDW_POWER));
            num += w * p[2];
            den += w;
        }
        return num / den;
    }

    // ---------- fitting ----------

    /** Basis terms for a fit of this degree: 1 (mean), 3 (plane) or 6 (curved surface). */
    private static double[] basis(int terms, double u, double v) {
        double[] all = {1, u, v, u * u, u * v, v * v};
        double[] b = new double[terms];
        System.arraycopy(all, 0, b, 0, terms);
        return b;
    }

    private void fitPoly(int degree) {
        int terms = degree == 0 ? 1 : (degree == 1 ? 3 : 6);
        // Need at least as many rows as terms; otherwise fall back to a simpler surface.
        while (terms > 1 && pts.size() < terms) { degree--; terms = degree == 0 ? 1 : (degree == 1 ? 3 : 6); }
        double[][] a = new double[terms][terms + 1];
        for (double[] p : pts) {
            double[] b = basis(terms, (p[0] - MID) / MID, (p[1] - MID) / MID);
            for (int i = 0; i < terms; i++) {
                for (int j = 0; j < terms; j++) a[i][j] += b[i] * b[j];
                a[i][terms] += b[i] * p[2];
            }
        }
        double[] sol = solve(a, terms, 1e-6); // a tiny ridge keeps rows in a straight line from breaking the fit
        coeffs = new double[6];
        if (sol == null) {
            double mean = 0;
            for (double[] p : pts) mean += p[2];
            coeffs[0] = mean / pts.size();
            degreeUsed = 0;
            return;
        }
        System.arraycopy(sol, 0, coeffs, 0, terms);
        degreeUsed = degree;
    }

    /** Gauss elimination on an m x (m+1) augmented matrix, with ridge added to the diagonal. Null if singular. */
    private static double[] solve(double[][] a, int m, double ridge) {
        for (int i = 0; i < m; i++) a[i][i] += ridge;
        for (int col = 0; col < m; col++) {
            int pivot = col;
            for (int r = col + 1; r < m; r++) if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
            double[] tmp = a[col]; a[col] = a[pivot]; a[pivot] = tmp;
            if (Math.abs(a[col][col]) < 1e-12) return null;
            for (int r = col + 1; r < m; r++) {
                double f = a[r][col] / a[col][col];
                for (int c = col; c <= m; c++) a[r][c] -= f * a[col][c];
            }
        }
        double[] x = new double[m];
        for (int i = m - 1; i >= 0; i--) {
            double sum = a[i][m];
            for (int j = i + 1; j < m; j++) sum -= a[i][j] * x[j];
            x[i] = sum / a[i][i];
        }
        return x;
    }

    private double computeR2() {
        double mean = 0;
        for (double[] p : pts) mean += p[2];
        mean /= pts.size();
        double tot = 0, res = 0;
        for (double[] p : pts) {
            double e = p[2] - evaluate(p[0], p[1]);
            res += e * e;
            tot += (p[2] - mean) * (p[2] - mean);
        }
        return tot < 1e-9 ? 1.0 : 1.0 - res / tot;
    }
}
