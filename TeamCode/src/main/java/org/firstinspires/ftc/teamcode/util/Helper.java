package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Vector2d;

public class Helper {
    public static double dist(Vector2d pos1, Vector2d pos2) {
        return Math.sqrt(Math.pow(pos2.x - pos1.x, 2) + Math.pow(pos2.y - pos1.y, 2));
    }
    public static double dist(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
    }
}
