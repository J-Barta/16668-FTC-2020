package org.firstinspires.ftc.teamcode.purePursuit;

public class mathFunctions {
    /**
     * Make sure angle is within 180 to -180 degrees
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle) {
       while(angle < -Math.PI) {
           angle += 2*Math.PI;
       }
       while(angle > Math.PI) {
           angle -= 2* Math.PI;
       }
       return angle;
    }
}
