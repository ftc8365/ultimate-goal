package org.firstinspires.ftc.teamcode.ultimategoal.robot;

public class PIDController {

    // Neverest 20 motors = 560 ticks per rotation
    // Neverest 40 moteos = 1120 tickers per rotation
    final int TICK_PER_WHEEL_ROTATION   = 560;
    final int TICK_PER_WHEEL_ODOMETRY_ROTATION = 7500;

    final double RAMP_UP_RATE_DRIVE     = 0.01;
    final double RAMP_UP_RATE_TURN      = 0.01;
    final double RAMP_UP_RATE_STRAFE    = 0.05;

    final double RAMP_DOWN_DRIVE_TICKS  = TICK_PER_WHEEL_ROTATION / 3.5;
    final double RAMP_DOWN_DRIVE_ODOMETRY_TICKS  = TICK_PER_WHEEL_ODOMETRY_ROTATION / 3.0;

    final double RAMP_DOWN_DRIVE_RANGE  = 30;
    final double RAMP_DOWN_TURN_DEGREES = 30;

    final double MIN_DRIVE_POWER        = 0.10;
    final double MIN_TURN_POWER         = 0.35;
    final double MIN_STRAFE_POWER       = 0.30;
    final double TURN_POWER             = 0.70;
    final double TURN_TOLERANCE         = 5.0;



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getDrivePower
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public double getSidewayDrivePower( double curPower, double ticksToGo, double targetPower, boolean rampDown ) {
        double power = curPower;

        // Ramp down power
        if ( rampDown && (ticksToGo <= RAMP_DOWN_DRIVE_ODOMETRY_TICKS) ) {
            power = ( ticksToGo / RAMP_DOWN_DRIVE_ODOMETRY_TICKS )  * curPower;

            if (power < MIN_STRAFE_POWER)
                power = MIN_STRAFE_POWER;
        }
        else if (targetPower - power > 0.001) {
            power += this.RAMP_UP_RATE_DRIVE;
        }

        return power;
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getDrivePower
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public double getDrivePower( double curPower, double ticksToGo, double targetPower, boolean rampDown ) {
        double power = curPower;

        // Ramp down power
        if ( rampDown && (ticksToGo <= RAMP_DOWN_DRIVE_TICKS) ) {
            power = ( ticksToGo / RAMP_DOWN_DRIVE_TICKS)  * curPower;

            if (power < this.MIN_DRIVE_POWER)
                power = MIN_DRIVE_POWER;

        } else if (rampDown && (curPower > targetPower)) {
            power -= (curPower - targetPower) / 5;
        }
        else if (targetPower - power > 0.001) {
            power += this.RAMP_UP_RATE_DRIVE;
        }

        return power;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getDrivePower
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public double getDriveRangePower( double curPower, double range, double targetPower ) {
        double power = curPower;

        // Ramp down power
        if ( range <= RAMP_DOWN_DRIVE_RANGE ) {
            power = ( range / RAMP_DOWN_DRIVE_RANGE)  * curPower;

            if (power < this.MIN_DRIVE_POWER)
                power = MIN_DRIVE_POWER;
        }
        else if (targetPower - power > 0.001) {
            power += this.RAMP_UP_RATE_DRIVE;
        }

        return power;
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getTurnPower
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public double getTurnPower( double curPower, double degreesToGo, double targetPower, boolean rampDown ) {
        double power = curPower;
        double RAMP_DOWN_DISTANCE = 30;

        // Ramp down power
        if ( rampDown && (degreesToGo <= RAMP_DOWN_DISTANCE) ) {
            power = ( degreesToGo / RAMP_DOWN_DISTANCE)  * curPower;

            if (power < MIN_TURN_POWER)
                power = MIN_TURN_POWER;
        }
        else if (targetPower - power > 0.001) {
            power += this.RAMP_UP_RATE_TURN;
        }
        return power;
    }

}
