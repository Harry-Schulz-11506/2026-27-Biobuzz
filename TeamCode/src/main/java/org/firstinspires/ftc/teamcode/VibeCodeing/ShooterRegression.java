package org.firstinspires.ftc.teamcode.VibeCodeing;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Distance -> flywheel RPM, built to be tuned live in FTC Dashboard (open "ShooterRegression").
 *
 * HOW TO TUNE
 *  1. Shoot from a spot. If it was short/long, edit that point's RPM (R1..R8) in Dashboard. It applies instantly.
 *  2. Need a new spot? Fill the next empty slot (distance D5 and RPM R5). A slot with D = 0 is unused.
 *     Or press L3 in TeleOp: it records the current distance + RPM into the next empty slot.
 *  3. Everything still off by the same amount? Change SHOT_OFFSET_RPM (adds to every shot).
 *  4. Pick how the points become a curve with FIT_TYPE (below). Start with 1 (connect the dots).
 *  5. Dashboard changes are lost on restart: copy the final values into the DEFAULT arrays and fields below.
 *
 * FIT_TYPE
 *  0 = polynomial least-squares fit through the points (degree = REGRESSION_DEGREE from MainTeleOp/AutoActions)
 *  1 = connect the dots: straight lines between your points, so every point is hit EXACTLY (best for tuning)
 *  2 = logarithmic fit   rpm = a + b*ln(d)    (flattens at long range)
 *  3 = power-law fit     rpm = a * d^b
 * Outside the first/last point, FIT_TYPE 1 holds the end RPM flat; the others are clamped to MIN/MAX RPM.
 */
@Config
public class ShooterRegression {

    // Measured shots (field inches -> flywheel RPM), recorded 26 Oct 2026. These seed the table below.
    public static final double[] DEFAULT_DIST = {31.6, 46.1, 54.5, 66.8, 48.9, 66.3};
    public static final double[] DEFAULT_RPM = {2900, 3200, 3200, 3300, 2815, 3300};

    // ---- Live-tunable options ----
    public static int FIT_TYPE = 0;              // see above (0 keeps the old behaviour)
    public static double SHOT_OFFSET_RPM = 0;    // added to every prediction

    // ---- Live-tunable shot table (D = distance in inches, R = RPM; D = 0 means the slot is empty) ----
    public static double D1 = 31.6, R1 = 2800;
    public static double D2 = 46.1, R2 = 3100;
    public static double D3 = 54.5, R3 = 3100;
    public static double D4 = 66.8, R4 = 3100;
    public static double D5 = 48.9, R5 = 2715; // 27 Oct 2026, robot (40.36, 5.72) to RED CELL 2
    public static double D6 = 66.3, R6 = 3200; // 27 Oct 2026, robot (8.61, 5.67) to RED CELL 2
    public static double D7 = 0,    R7 = 0;
    public static double D8 = 0,    R8 = 0;

    private static final int SLOTS = 8;

    private int requestedDegree;
    private int degreeUsed = 0;
    private double rSquared = 1.0;
    private double[] coeffs = new double[3]; // polynomial: rpm = c0 + c1*d + c2*d^2 ; log/power: a, b
    private List<double[]> pts = new ArrayList<>();

    public ShooterRegression(int degree) {
        refit(degree);
    }

    // ---------- table access ----------

    private static double[] slot(int i) {
        switch (i) {
            case 0: return new double[]{D1, R1};
            case 1: return new double[]{D2, R2};
            case 2: return new double[]{D3, R3};
            case 3: return new double[]{D4, R4};
            case 4: return new double[]{D5, R5};
            case 5: return new double[]{D6, R6};
            case 6: return new double[]{D7, R7};
            default: return new double[]{D8, R8};
        }
    }

    private static void setSlot(int i, double d, double r) {
        switch (i) {
            case 0: D1 = d; R1 = r; break;
            case 1: D2 = d; R2 = r; break;
            case 2: D3 = d; R3 = r; break;
            case 3: D4 = d; R4 = r; break;
            case 4: D5 = d; R5 = r; break;
            case 5: D6 = d; R6 = r; break;
            case 6: D7 = d; R7 = r; break;
            default: D8 = d; R8 = r; break;
        }
    }

    /** Points currently in the table, sorted by distance (empty slots skipped). */
    private static List<double[]> readTable() {
        List<double[]> list = new ArrayList<>();
        for (int i = 0; i < SLOTS; i++) {
            double[] s = slot(i);
            if (s[0] > 0 && s[1] > 0) list.add(s);
        }
        Collections.sort(list, new Comparator<double[]>() {
            @Override public int compare(double[] a, double[] b) { return Double.compare(a[0], b[0]); }
        });
        return list;
    }

    // ---------- API used by MainTeleOp / AutoActions ----------

    public boolean usingRecorded() { return recordedCount() > 0; }
    public int recordedCount() { return Math.max(0, readTable().size() - DEFAULT_DIST.length); }
    public int getRequestedDegree() { return requestedDegree; }
    public int getDegreeUsed() { return degreeUsed; }
    public double getRSquared() { return rSquared; }
    public double[] getCoefficients() { return coeffs.clone(); }

    /**
     * Adds a shot to the table: the first empty slot, or, if all 8 are used, it replaces the point
     * nearest in distance. It then shows up in Dashboard where you can keep editing it.
     */
    public void addSample(double distance, double rpm) {
        for (int i = 0; i < SLOTS; i++) {
            if (slot(i)[0] <= 0) { setSlot(i, distance, rpm); refit(requestedDegree); return; }
        }
        int nearest = 0;
        for (int i = 1; i < SLOTS; i++) {
            if (Math.abs(slot(i)[0] - distance) < Math.abs(slot(nearest)[0] - distance)) nearest = i;
        }
        setSlot(nearest, distance, rpm);
        refit(requestedDegree);
    }

    /** Puts the table back to the 4 built-in measured shots and clears the offset. */
    public void clearRecorded() {
        for (int i = 0; i < SLOTS; i++) {
            if (i < DEFAULT_DIST.length) setSlot(i, DEFAULT_DIST[i], DEFAULT_RPM[i]);
            else setSlot(i, 0, 0);
        }
        SHOT_OFFSET_RPM = 0;
        refit(requestedDegree);
    }

    /** The table as "dist:rpm" pairs, for copying into the defaults. */
    public String describeSamples() {
        StringBuilder sb = new StringBuilder();
        for (double[] s : readTable()) {
            if (sb.length() > 0) sb.append(", ");
            sb.append(String.format("%.1f:%.0f", s[0], s[1]));
        }
        return sb.toString();
    }

    /** How far the current curve is from each point, e.g. "31.6:+12, 46.1:-30". Use this to see which point to nudge. */
    public String describeErrors() {
        StringBuilder sb = new StringBuilder();
        for (double[] s : pts) {
            if (sb.length() > 0) sb.append(", ");
            sb.append(String.format("%.1f:%+.0f", s[0], evaluate(s[0]) - s[1]));
        }
        return sb.toString();
    }

    /** Predicted RPM for a distance, plus SHOT_OFFSET_RPM, clamped to [minRpm, maxRpm]. Reads the live table every call. */
    public double predict(double distance, double minRpm, double maxRpm) {
        refit(requestedDegree); // picks up Dashboard edits immediately (at most 8 points, very cheap)
        double y = evaluate(distance) + SHOT_OFFSET_RPM;
        return Math.max(minRpm, Math.min(maxRpm, y));
    }

    /** Re-read the table and re-run the fit. degree only matters for FIT_TYPE 0 (limited to points - 1). */
    public void refit(int degree) {
        requestedDegree = Math.max(0, Math.min(2, degree));
        pts = readTable();
        coeffs = new double[3];
        degreeUsed = 0;
        rSquared = 1.0;
        if (pts.isEmpty()) return;

        if (FIT_TYPE == 1) {
            degreeUsed = 1;
            return; // exact through every point
        }
        if (pts.size() == 1) {
            coeffs[0] = pts.get(0)[1];
            return;
        }
        if (FIT_TYPE == 2) {
            fitLinear(transform(pts, true, false));
        } else if (FIT_TYPE == 3) {
            fitLinear(transform(pts, true, true));
        } else {
            fitPoly(Math.min(requestedDegree, pts.size() - 1));
        }
        rSquared = computeR2();
    }

    // ---------- evaluation ----------

    private double evaluate(double d) {
        if (pts.isEmpty()) return 0;
        if (FIT_TYPE == 1) return interpolate(d);
        if (pts.size() == 1) return coeffs[0];
        d = Math.max(d, 1e-3);
        if (FIT_TYPE == 2) return coeffs[0] + coeffs[1] * Math.log(d);
        if (FIT_TYPE == 3) return Math.exp(coeffs[0] + coeffs[1] * Math.log(d));
        return coeffs[0] + coeffs[1] * d + coeffs[2] * d * d;
    }

    private double interpolate(double d) {
        double[] first = pts.get(0), last = pts.get(pts.size() - 1);
        if (d <= first[0]) return first[1];
        if (d >= last[0]) return last[1];
        for (int i = 0; i < pts.size() - 1; i++) {
            double[] a = pts.get(i), b = pts.get(i + 1);
            if (d <= b[0]) {
                double span = b[0] - a[0];
                if (span < 1e-9) return b[1];
                return a[1] + (b[1] - a[1]) * (d - a[0]) / span;
            }
        }
        return last[1];
    }

    // ---------- fitting ----------

    /** Builds (x, y) pairs for the log / power fits: x = ln(d), y = rpm or ln(rpm). */
    private static List<double[]> transform(List<double[]> in, boolean logX, boolean logY) {
        List<double[]> out = new ArrayList<>();
        for (double[] p : in) {
            out.add(new double[]{logX ? Math.log(p[0]) : p[0], logY ? Math.log(p[1]) : p[1]});
        }
        return out;
    }

    /** y = a + b*x by least squares; stored in coeffs[0], coeffs[1]. */
    private void fitLinear(List<double[]> xy) {
        int n = xy.size();
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (double[] p : xy) { sx += p[0]; sy += p[1]; sxx += p[0] * p[0]; sxy += p[0] * p[1]; }
        double den = n * sxx - sx * sx;
        double b = Math.abs(den) < 1e-9 ? 0 : (n * sxy - sx * sy) / den;
        coeffs[0] = (sy - b * sx) / n;
        coeffs[1] = b;
        degreeUsed = 1;
    }

    private void fitPoly(int degree) {
        int m = degree + 1;
        double[][] a = new double[m][m + 1];
        for (double[] p : pts) {
            for (int i = 0; i < m; i++) {
                for (int j = 0; j < m; j++) a[i][j] += Math.pow(p[0], i + j);
                a[i][m] += p[1] * Math.pow(p[0], i);
            }
        }
        for (int col = 0; col < m; col++) {
            int pivot = col;
            for (int r = col + 1; r < m; r++) if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
            double[] tmp = a[col]; a[col] = a[pivot]; a[pivot] = tmp;
            if (Math.abs(a[col][col]) < 1e-9) {
                if (degree > 0) fitPoly(degree - 1); // singular (all points at one distance): use a lower degree
                return;
            }
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
        coeffs = new double[3];
        for (int i = 0; i < m; i++) coeffs[i] = x[i];
        degreeUsed = degree;
    }

    private double computeR2() {
        double mean = 0;
        for (double[] p : pts) mean += p[1];
        mean /= pts.size();
        double tot = 0, res = 0;
        for (double[] p : pts) {
            double e = p[1] - evaluate(p[0]);
            res += e * e;
            tot += (p[1] - mean) * (p[1] - mean);
        }
        return tot < 1e-9 ? 1.0 : 1.0 - res / tot;
    }
}