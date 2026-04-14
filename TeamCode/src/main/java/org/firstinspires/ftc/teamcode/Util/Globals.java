package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class Globals {
    public static Pose autoEndPose = new Pose(72,7.81, Math.toRadians(90)); // default for easier testing
    public static double blueGoalX = 7.5;
    public static double blueGoalY = 139.5;
    public static double redGoalX = 145.5;
    public static double redGoalY = 139.5;
    public static double PERMANENT_blueGoalX = 7.5;
    public static double PERMANENT_blueGoalY = 139.5;
    public static double PERMANENT_redGoalX = 145.5;
    public static double PERMANENT_redGoalY = 139.5;
    public static boolean blueAlliance = true;
    public static double MINIMUM_LOOP_TIME = 11;
}