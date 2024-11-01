package com.stuypulse.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/** Huge thanks to Elin for doing all the math */ 
public class SpeakerAngleElinInterpolation {

    private static final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap;

    // distance (angle), angle (radians)
    private static final double[][] distanceAndAngle = {
        // {45.1854245,-0.57751},
        // {47.5, -0.60548},
        // {50, -0.63458},
        // {52.5, -0.66255},
        // {55, -0.68943},
        // {57.5,-0.71524},
        // {60, -0.74002},
        // {62.5, -0.76382},
        // {65, -0.78668},
        // {67.5, -0.80862},
        // {70, -0.82969},
        // {72.5, -0.84993},
        // {75, -0.86938},
        // {77.5, -0.88806},
        // {80, -0.90602},
        // {82.5, -0.92328},
        // {85, -0.93989},
        // {87.5, -0.95587},
        // {90, -0.97125},
        // {92.5, -0.98606},
        // {95, -1.00032},
        // {97.5, -1.01407},
        // {100, -1.02732},
        // {102.5, -1.0401},
        // {105, -1.05243},
        // {107.5, -1.06434},
        // {110, -1.07583},
        // {112.5, -1.08693},
        // {115, -1.09767},
        // {117.5, -1.10804},
        // {120, -1.11808},
        // {122.5, -1.12779},
        // {125, -1.13719},
        // {127.5, -1.14629},
        // {130, -1.15512},
        // {132.5, -1.16366},
        // {135, -1.17195},
        // {137.5, -1.17999},
        // {140, -1.18779},
        // {142.5, -1.19536},
        // {145, -1.20271},
        // {147.5, -1.20986},
        // {150, -1.21679},
        // {152.5, -1.22354},
        // {155, -1.2301},
        // {157.5, -1.23647},
        // {160, -1.24268},
        // {162.5, -1.24872},
        // {165, -1.2546},
        // {167.5, -1.26033},
        // {170, -1.26591},
        // {172.5, -1.27134},
        // {175, -1.27664},
        // {177.5, -1.28181},
        // {180, -1.28685},
        {45, -0.53055},
        {47.5, -0.56248},
        {50, -0.59322},
        {52.5, -0.62279},
        {55, -0.65122},
        {57.5, -0.67854},
        {60, -0.70478},
        {62.5, -0.72998},
        {65, -0.75418},
        {67.5, -0.77742},
        {70, -0.79973},
        {72.5, -0.82116},
        {75, -0.84173},
        {77.5, -0.8615},
        {80, -0.88049},
        {82.5, -0.89874},
        {85, -0.91629},
        {87.5, -0.93316},
        {90, -0.94939},
        {92.5, -0.96502},
        {95, -0.98005},
        {97.5, -0.99454},
        {100, -1.00849},
        {102.5, -1.02195},
        {105, -1.03492},
        {107.5, -1.04743},
        {110, -1.05951},
        {112.5, -1.07117},
        {115, -1.08243},
        {117.5, -1.09331},
        {120, -1.10383},
        {122.5, -1.114},
        {125, -1.12385},
        {127.5, -1.13337},
        {130, -1.1426},
        {132.5, -1.15153},
        {135, -1.16019},
        {137.5, -1.16858},
        {140, -1.17672},
        {142.5, -1.18462},
        {145, -1.19229},
        {147.5, -1.19973},
        {150, -1.20695},
        {152.5, -1.21397},
        {155, -1.2208},
        {157.5, -1.22743},
        {160, -1.23388},
        {162.5, -1.24016},
        {165, -1.24626},
        {167.5, -1.25221},
        {170, -1.258},
        {172.5, -1.26364},
        {175, -1.26913},
        {177.5, -1.27449},
        {180, -1.27971},
        {182.5, -1.2848},
        {185, -1.28976},
        {187.5, -1.29461},
        {190, -1.29934},
        {192.5, -1.30396},
        {195, -1.30846},
        {197.5, -1.31287},
        {200, -1.31717},
        {202.5, -1.32137},
        {205, -1.32548},
        {207.5, -1.3295},
        {210, -1.33343},
        {212.5, -1.33727},
        {215, -1.34103},
        {217.5, -1.34471},
        {220, -1.34831},
        {222.5, -1.35184},
        {225, -1.35529},
        {227.5, -1.35867},
        {230, -1.36198},
    };

    static {
        interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();
        for (double[] data: distanceAndAngle) {
            interpolatingDoubleTreeMap.put(data[0] - 1.4375, data[1]);
        }
    }

    public static double getAngleInRadians(double distanceInInches) {
        return interpolatingDoubleTreeMap.get(distanceInInches);
    }

    public static double getAngleInDegrees(double distanceInInches) {
        return Units.radiansToDegrees(getAngleInRadians(distanceInInches));
    }
}
