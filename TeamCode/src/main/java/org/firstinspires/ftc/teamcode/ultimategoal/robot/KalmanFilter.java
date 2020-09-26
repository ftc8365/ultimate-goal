package org.firstinspires.ftc.teamcode.ultimategoal.robot;

public class KalmanFilter {
    double lastValue = 0;
    double delta = 0;
    double currentValue = 0;
    boolean hasValidData = false;

    void reset() {
        lastValue = 0;
        delta = 0;
        currentValue = 0;
        hasValidData = false;
    }

    boolean hasValidEstimate() {
        return hasValidData;
    }

    double getEstimate() {
        return lastValue > 0 ? currentValue + (currentValue - lastValue)  : currentValue;
    }

    void updateValue( double value) {
        if (value != currentValue) {
            lastValue = currentValue;
            currentValue = value;
        }
    }
}
