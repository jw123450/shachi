package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class Globals {
    public static Pose autoEndPose = new Pose(72,8.5, Math.toRadians(90)); // default for easier testing
    public static double blueGoalX = 10.5; // TODO: tune locations depending preferred backboard aim
    public static double blueGoalY = 139; // TODO
    public static double redGoalX = 134.5; // TODO
    public static double redGoalY = 139; // TODO
    public static boolean blueAlliance = true;
}