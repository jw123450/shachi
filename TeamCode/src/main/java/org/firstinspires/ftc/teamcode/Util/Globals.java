package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class Globals {
    public static Pose autoEndPose = new Pose(72,7.81, Math.toRadians(90)); // default for easier testing
    public static double blueGoalX = 15.5; // TODO: tune locations depending preferred backboard aim //old: 10.5
    public static double blueGoalY = 136; // TODO //old: 139
    public static double redGoalX = 140; // TODO //old: 134.5
    public static double redGoalY = 136; // TODO //old: 139
    public static boolean blueAlliance = true;
}