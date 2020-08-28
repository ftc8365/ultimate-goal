package org.firstinspires.ftc.teamcode.skystone2;

public class MathFunctions {
    /*
        Makes sure the angle is between -180 to 180 degrees
     */
    public static double angleWrap( double angle ) {
        while (angle < - Math.PI) {
            angle += 2 * Math.PI;
        }

        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }
}
