package org.altbeacon.beacon.service;

/**
 * Calculate a RSSI value by using a 1-dimensional simplified Kalman filter. Because it is assumed
 * that the device doesn't move in a short time frame, a simplification of the state transformation
 * matrix and control vector were done.
 *
 *     Based on the details of:
 *     https://wouterbulten.nl/blog/tech/kalman-filters-explained-removing-noise-from-rssi-signals/
 *
 */
public class KalmanRssiFilter implements RssiFilter {

    private final double R = 0.01;      /* Process noise */
    private final double Q = 15;        /* Measurement noise */
    private final double A = 1;         /* State transitioning model */
    private final double C = 1;         /* Observation transitioning model */
    private final double B = 0;         /* Control multiplication vector */

    private double currentState = 0;    /* Estimated RSSI without noise */
    private double cov = 0;             /* Covariance */

    @Override
    public void addMeasurement(Integer rssi) {
        double predState;   // Predicted state
        double predCov;     // Predicted covariance
        double K;           // Kalman gain
        double u = 0;       // Control input

        if (currentState == 0) {
            currentState = (1 / C) * rssi;
            cov = (1 / C) * Q * (1 / C);
        } else {

            // Compute prediction
            predState = (A * currentState) + (B * u);
            predCov = ((A * cov ) * A) + R;

            // Compute Kalman gain
            K = predCov * C * (1 / ((C * predCov * C) + Q));

            // Apply correction
            currentState = predState + K * (rssi - (C * predState));
            cov = predCov - (K * C * predCov);
        }
    }

    @Override
    public boolean noMeasurementsAvailable() {
        return (currentState == 0);
    }

    @Override
    public double calculateRssi() {
        return currentState;
    }
}
