package org.firstinspires.ftc.teamcode.VibeCodeing;

import java.util.ArrayList;
import java.util.List;

/**
 * Distance -> flywheel RPM regression (least-squares polynomial fit, degree 1 = line, 2 = curve).
 *
 * Two data sets:
 *  - DEFAULT_*: the measured shot table below (recorded 26 Oct 2026).
 *  - recorded samples: added at runtime with addSample() (MainTeleOp records one on left stick button).
 *    Once 2 or more are recorded they replace the default table.
 *
 * To make recorded samples permanent, copy the "samples" line from telemetry into the DEFAULT arrays.
 */
public class ShooterRegression {

    // Measured shots (field inches from the target, flywheel RPM target), recorded 26 Oct 2026.
    // These distances were the straight line from the robot pose to the point (60, 48); the code now
    // measures to whichever goal cell the turret has selected, and the same table is used for all of them:
    //   robot (41.01, 22.76) -> 31.6 in @ 2900 RPM
    //   robot (64.01,  2.12) -> 46.1 in @ 3200 RPM
    //   robot (25.73,  5.64) -> 54.5 in @ 3200 RPM
    //   robot ( 6.41,  8.08) -> 66.8 in @ 3300 RPM
    public static final double[] DEFAULT_DIST = {31.6, 46.1, 54.5, 66.8};
    public static final double[] DEFAULT_RPM = {2900, 3200, 3200, 3300};

    private final List<double[]> recorded = new ArrayList<>();
    private double[] coeffs = new double[3]; // rpm = c0 + c1*d + c2*d^2
    private int requestedDegree;
    private int degreeUsed = 0;
    private double rSquared = 0;

    public ShooterRegression(int degree) {
        refit(degree);
    }

    private List<double[]> activeSamples() {
        if (recorded.size() >= 2) return recorded;
        List<double[]> list = new ArrayList<>();
        for (int i = 0; i < DEFAULT_DIST.length; i++) {
            list.add(new double[]{DEFAULT_DIST[i], DEFAULT_RPM[i]});
        }
        return list;
    }

    public boolean usingRecorded() { return recorded.size() >= 2; }
    public int recordedCount() { return recorded.size(); }
    public int getRequestedDegree() { return requestedDegree; }
    public int getDegreeUsed() { return degreeUsed; }
    public double getRSquared() { return rSquared; }
    public double[] getCoefficients() { return coeffs.clone(); }

    public void addSample(double distance, double rpm) {
        recorded.add(new double[]{distance, rpm});
        refit(requestedDegree);
    }

    public void clearRecorded() {
        recorded.clear();
        refit(requestedDegree);
    }

    /** Samples as "dist:rpm" pairs, for copying into the DEFAULT arrays. */
    public String describeSamples() {
        StringBuilder sb = new StringBuilder();
        for (double[] s : activeSamples()) {
            if (sb.length() > 0) sb.append(", ");
            sb.append(String.format("%.0f:%.0f", s[0], s[1]));
        }
        return sb.toString();
    }

    /** Predicted RPM for a distance, clamped to [minRpm, maxRpm]. */
    public double predict(double distance, double minRpm, double maxRpm) {
        double y = coeffs[0] + coeffs[1] * distance + coeffs[2] * distance * distance;
        return Math.max(minRpm, Math.min(maxRpm, y));
    }

    /** Re-run the least-squares fit with the given polynomial degree (limited to samples - 1). */
    public void refit(int degree) {
        requestedDegree = Math.max(0, Math.min(2, degree));
        List<double[]> pts = activeSamples();
        if (pts.isEmpty()) return;
        fit(pts, Math.min(requestedDegree, pts.size() - 1));
    }

    private void fit(List<double[]> pts, int degree) {
        int m = degree + 1;
        double[][] a = new double[m][m + 1];
        for (double[] p : pts) {
            double d = p[0];
            double y = p[1];
            for (int i = 0; i < m; i++) {
                for (int j = 0; j < m; j++) {
                    a[i][j] += Math.pow(d, i + j);
                }
                a[i][m] += y * Math.pow(d, i);
            }
        }

        // Gaussian elimination with partial pivoting.
        for (int col = 0; col < m; col++) {
            int pivot = col;
            for (int r = col + 1; r < m; r++) {
                if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
            }
            double[] tmp = a[col];
            a[col] = a[pivot];
            a[pivot] = tmp;
            if (Math.abs(a[col][col]) < 1e-9) {
                if (degree > 0) {
                    fit(pts, degree - 1); // singular (for example all samples at one distance): use a lower degree
                }
                return;
            }
            for (int r = col + 1; r < m; r++) {
                double f = a[r][col] / a[col][col];
                for (int c = col; c <= m; c++) {
                    a[r][c] -= f * a[col][c];
                }
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

        // R^2 as a quick quality check.
        double mean = 0;
        for (double[] p : pts) mean += p[1];
        mean /= pts.size();
        double ssTot = 0;
        double ssRes = 0;
        for (double[] p : pts) {
            double yHat = coeffs[0] + coeffs[1] * p[0] + coeffs[2] * p[0] * p[0];
            ssRes += (p[1] - yHat) * (p[1] - yHat);
            ssTot += (p[1] - mean) * (p[1] - mean);
        }
        rSquared = ssTot < 1e-9 ? 1.0 : 1.0 - ssRes / ssTot;
    }
}
