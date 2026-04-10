package org.firstinspires.ftc.teamcode.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Configurable
public class PinpointManager { // pinpoint for use during teleop, pedro has its own implementation
    GoBildaPinpointDriver odo;
    OpMode opmode;
    Telemetry telemetry;

    double oldTime;

    // all of the following are in degrees
    public volatile double absoluteHeading, normalizedHeading, relativeNormalizedHeading, offset, X, Y, velX, velY;
    public volatile Vector velVector;

    public PinpointManager() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        this.telemetry = opmode.telemetry;

        odo = robotHardware.odo;

        odo.resetPosAndIMU();

        offset = 0;

        opmode.telemetry.addData("Status", "Initialized");
        opmode.telemetry.update();
    }

    public void operateTesting() {
        odo.update();

        absoluteHeading = odo.getHeading(AngleUnit.DEGREES);
        normalizedHeading = normalize(absoluteHeading);
        relativeNormalizedHeading = normalize(normalizedHeading - offset);

        telemetry.addData("UNnormalized absolute heading (degrees)", absoluteHeading);
        telemetry.addData("normalized absolute heading (degrees)", normalizedHeading);
        telemetry.addData("normalized relative heading (degrees)", relativeNormalizedHeading);
        telemetry.addData("offset (degrees)", offset);

        // loop times
        double newTime = opmode.getRuntime();
        double frequency = 1/(newTime-oldTime);
        oldTime = newTime;
        telemetry.addData("Pinpoint Frequency (Hz)", odo.getFrequency()); // pinpoint refresh rate
        telemetry.addData("REV Hub Frequency (Hz): ", frequency); // control hub refresh rate

//        Pose2D pos = odo.getPosition();
//        String data = String.format(Locale.US, "{X: %.3f mm, Y: %.3f mm, Heading: %.3f°}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("Position", data);
//
//        Pose2D vel = odo.getVelocity();
//        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("Velocity", velocity);
//        telemetry.addData("Status", odo.getDeviceStatus());
//        telemetry.update();
    }

    public void operateTrackingPose() {
        odo.update();
        Pose2D currentPose = odo.getPosition();

        velX = odo.getVelX(DistanceUnit.INCH);
        velY = odo.getVelY(DistanceUnit.INCH); // inch/sec
//        velVector = new Vector();
//        velVector.setOrthogonalComponents(velX, velY);

//        absoluteHeading = odo.getHeading(AngleUnit.DEGREES);
//        normalizedHeading = normalize(absoluteHeading);
        normalizedHeading = normalize(currentPose.getHeading(AngleUnit.DEGREES));

//        X = odo.getPosX(DistanceUnit.INCH) + 72; // converts x and y to pedro field, not gobilda's
//        Y = odo.getPosY(DistanceUnit.INCH) + 72;
        X = currentPose.getX(DistanceUnit.INCH) + 72; // converts x and y to pedro field, not gobilda's
        Y = currentPose.getY(DistanceUnit.INCH) + 72;
    }

    public void operateSimple() {
        odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

        normalizedHeading = odo.getHeading(AngleUnit.DEGREES);
        relativeNormalizedHeading = normalize(normalizedHeading - offset);
    }

    public void transferAutoPose(Pose autoEndPose) {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, autoEndPose.getX() - 72, autoEndPose.getY()  - 72, AngleUnit.RADIANS, autoEndPose.getHeading()));
    }

    public void teleOpManualReset(boolean blueAlliance) {
        if (blueAlliance) {                                 // right side field is 63.5, -62
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 63.1, -64.8, AngleUnit.DEGREES, 180));
        } else {                                            // right side field is 63.5, -62
            odo.setPosition(new Pose2D(DistanceUnit.INCH, -63.1, -64.8, AngleUnit.DEGREES, 0));
        }
    }

    public void teleOpResetHeading() {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, X-72, Y-72, AngleUnit.DEGREES, 90));
    }

    public void teleOpAprilTagReset(Pose currentPoseLL, boolean tag24red) {
        // currentPoseLL is already in pinpoint-style coords.
        if (tag24red) {
            odo.setPosition(new Pose2D(DistanceUnit.INCH, (currentPoseLL.getX() + 3.5), (currentPoseLL.getY() + 3), AngleUnit.RADIANS, currentPoseLL.getHeading()));
        } else {
            odo.setPosition(new Pose2D(DistanceUnit.INCH, (currentPoseLL.getX() - 1), (currentPoseLL.getY() + 3), AngleUnit.RADIANS, currentPoseLL.getHeading()));
        }
    }

    public void softResetYaw() {
        offset += normalizedHeading;
    }

    public double normalize(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public double getFreq() {
        return odo.getFrequency();
    }

    public double getLT() {
        return odo.getLoopTime() * 1000;
    }
}