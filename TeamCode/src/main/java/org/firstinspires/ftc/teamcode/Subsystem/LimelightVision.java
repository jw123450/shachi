package org.firstinspires.ftc.teamcode.Subsystem;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Util.Globals;
import org.firstinspires.ftc.teamcode.Util.PinpointManager;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;


import java.util.List;


public class LimelightVision {
    private OpMode opmode;

    private double tx;

    private double ty;

    private LLStatus status;

    private LLResult result;
   // private LLResultTypes.FiducialResult fiducials;
    private int apriltag;

    private static double llMountAngle = 23; // degrees from vertical (pitch)

    private static double M2IFactor = 39.3700787; // inch/meter

    private static double llMountHeight = 10.57; // inches above floor == 268.492 mm

    private static double targetHeight = 39; // == 990.6 mm

    private static double forwardDistFromCenter = 8.11; // # inches in front of center of bot == 205.935 mm

    private Limelight3A ll3a;
//    private PinpointManager pinpoint;

    public boolean tagSeen = false;

    public void initialize(OpMode opmode, RobotHardware rHardware) {
        this.ll3a = rHardware.limelight;
        this.opmode = opmode;
        ll3a.start();
    }

    public void trackPose(boolean blueAlliance) {
        this.status = ll3a.getStatus();

        this.result = ll3a.getLatestResult();
      //  this.apriltag = result.getFiducialResults().get(0).getFiducialId(); //from limelight docs, blue is 20 and red is 24 for future reference
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                apriltag = fiducial.getFiducialId();
                if (apriltag == 20 || apriltag == 24) { // red or blue side in view
                    tagSeen = true;
                }
                if (apriltag == (blueAlliance ? 20 : 24)) { // alliance side in view
                    this.tx = result.getTx();
                    this.ty = result.getTy();
                }
            }
        } else {
            tagSeen = false;
        }
    }

//    public double getTargetAngle() {
//        // Using relative heading + offset
//        if (tx < 0) {
//            return pinpoint.relativeNormalizedHeading - tx;
//        } else if (tx > 0) {
//            return pinpoint.relativeNormalizedHeading + tx;
//        }
//        else {
//            return pinpoint.relativeNormalizedHeading;
//        }
//    }

    public double readDistance() {

        double targetOffsetVA = -this.ty;

        double angleToGoalDegrees = llMountAngle + targetOffsetVA;
        double angleToGoalRads = angleToGoalDegrees * (Math.PI / 180);

        return (targetHeight - llMountHeight) / Math.tan(angleToGoalRads);
    }

    public double getTx() {
        return tx;
    }

    public boolean tagDetected(boolean blueAlliance) { return result.isValid() && (apriltag == (blueAlliance ? 20 : 24)); }

    public Pose absRelocalize(double yawRads) {
        // already checked that tag seen
        if (result != null && result.isValid()) { // should already be checked for in trackPose() loop, but just in case
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().y; // in meters
                double y = -botpose.getPosition().x; // in meters
                // +pi/2 converts heading to pinpoint-style coordinates
                return new Pose(x * M2IFactor, y * M2IFactor, yawRads);

            }
        }
        opmode.telemetry.addLine("LL RESULT NULL"); // should never reach this line
        opmode.telemetry.addLine("LL RESULT NULL"); // should never reach this line
        return new Pose(1, 1, 1); // should never reacch this line
    }
}
