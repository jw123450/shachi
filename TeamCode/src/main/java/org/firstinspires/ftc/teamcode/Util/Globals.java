package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class Globals {
    public static Pose autoEndPose = new Pose(72,72, Math.toRadians(90)); // default for easier testing
    public static double blueGoalX = 6.5; //7.5
    public static double blueGoalY = 143; //139.5
    public static double redGoalX = 144; //145.5
    public static double redGoalY = 144; //139.5
    public static double PERMANENT_blueGoalX = 10;
    public static double PERMANENT_blueGoalY = 137;
    public static double PERMANENT_redGoalX = 140;
    public static double PERMANENT_redGoalY = 137;
    public static boolean blueAlliance = true;
    public static double MINIMUM_LOOP_TIME = 9;
}