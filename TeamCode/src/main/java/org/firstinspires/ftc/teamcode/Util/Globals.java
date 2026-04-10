package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class Globals {
    public static Pose autoEndPose = new Pose(72,7.81, Math.toRadians(90)); // default for easier testing
    public static double blueGoalX = 10.5; // TODO: tune locations depending preferred backboard aim //old: 15.5
    public static double blueGoalY = 139.5; // TODO //old: 136
    public static double redGoalX = 143.5; // TODO //old: 140
    public static double redGoalY = 139.5; // TODO //old: 136
    public static boolean blueAlliance = true;
    public static double MINIMUM_LOOP_TIME = 15;
}