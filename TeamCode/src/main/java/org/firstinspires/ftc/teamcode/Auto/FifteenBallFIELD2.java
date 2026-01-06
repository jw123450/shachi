package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.callbacks.PathCallback;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.BrakePad;
import org.firstinspires.ftc.teamcode.Subsystem.FlywheelShooter;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.LimelightVision;
import org.firstinspires.ftc.teamcode.Subsystem.Turret;
import org.firstinspires.ftc.teamcode.Util.Globals;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Util.RGBLights;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "15 + 3 FIELD 2222222222222", group = "A", preselectTeleOp = "Odom TeleOp")
public class FifteenBallFIELD2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private List<LynxModule> allHubs;
    private ElapsedTime elapsedtime;


    private RobotHardware robotHardware = new RobotHardware();
    private Intake intake = new Intake();
    private Turret turret = new Turret();
    private FlywheelShooter shooter = new FlywheelShooter();
    private LimelightVision llVision = new LimelightVision();
    private RGBLights lights = new RGBLights();
    private BrakePad brake = new BrakePad();


    ///  CONSTANTS
    private boolean blueAlliance = true;
    private int ballsLaunched = 0;
    private volatile boolean runShooter = false;
    private volatile boolean runTurret = false;
    private volatile boolean cyclingFarZone = true;
    private final double PUSHER_DELAY = 0.5; // seconds (sleepAction)
    private final double RAPID_FIRE_DELAY = 0.6; // seconds (sleepAction)
    private final double DELAY_BEFORE_MOVING = 125; // milliseconds
    private final double WAIT_GATE_DELAY = 3600; /// TUNE
    private final double SLOW_PATH_MAX_POWER = 0.65;
    private final double SLOW_BRAKE_STRENGTH = 1.2;
    private final double AT_POSE_X_TOLERANCE = 2.5;
    private final double AT_POSE_Y_TOLERANCE = 2.5;


    /// BLUE SIDE
    // Preload + Partner Push                             // 8.6?
    private final Pose startPoseBlue = new Pose(57.35, 9.5, Math.toRadians(90)); // 2.375
    private final Pose partnerPushPoseBlue = new Pose(52, 9, Math.toRadians(90));
    // Spike 2
    private final Pose travelS2PoseBlue = new Pose(40.5 ,56, Math.toRadians(155));
    private final Pose travelS2Control1Blue = new Pose(56.55, 33.6);
    private final Pose travelS2Control2Blue = new Pose(50.7, 51.55);
    private final Pose pickupS2PoseBlue = new Pose(23.5, 56, Math.toRadians(155)); /// slow down + linear heading
    private final Pose scoreS2PoseBlue = new Pose(60, 75, Math.toRadians(-143));
    private final Pose scoreS2Control1Blue = new Pose(29.2, 53.4);
    // Open Gate + Grab
    private final Pose openGatePoseBlue = new Pose(14.3, 60.5, Math.toRadians(140));
    private final Pose openGateControl1Blue = new Pose(46.5, 65.3);
    private final Pose openGateControl2Blue = new Pose(31.7, 41.7);
    private final Pose scoreGatePoseBlue = new Pose(59, 19.6, Math.toRadians(103));
    private final Pose scoreGateControl1Blue = new Pose(22, 49);
    private final Pose scoreGateControl2Blue = new Pose(47.8, 46.5);
    // Spike 3
    private final Pose travelS3PoseBlue = new Pose(43.6, 36, Math.toRadians(180));
    private final Pose travelS3Control1Blue = new Pose(51.6, 35);
    private final Pose pickupS3PoseBlue = new Pose(23.5, 36, Math.toRadians(180)); /// slow down + linear heading
    private final Pose scoreS3PoseBlue = new Pose(62, 20.6, Math.toRadians(134));
    private final Pose scoreS3Control1Blue = new Pose(46.9, 36.2);
    // HP Station
    private final Pose travelHPPoseBlue = new Pose(11.5, 22.75, Math.toRadians(-116));
    private final Pose travelHPControl1Blue = new Pose(38.2, 45.5);
    private final Pose travelHPControl2Blue = new Pose(14.6, 30);
    private final Pose pickupHPPoseBlue = new Pose(8, 11.3, Math.toRadians(260)); /// slow down + linear heading
    private final Pose scoreHPPoseBlue = new Pose(57, 78, Math.toRadians(160));
    private final Pose scoreHPControl1Blue = new Pose(12.5, 38.8);
    private final Pose scoreHPControl2Blue = new Pose(41.1, 83.9);
    // Spike 1
    private final Pose pickupS1PoseBlue = new Pose(21, 84, Math.toRadians(180)); /// maybe slow?
    private final Pose pickupS1Control1Blue = new Pose(38.9, 84);
    private final Pose scoreS1PoseBlue = new Pose(60, 100, Math.toRadians(229));
    private final Pose scoreS1Control1Blue = new Pose(46, 84);

    /// RED SIDE
    // Preload + Partner Push
    private final Pose startPoseRed = startPoseBlue.mirror();
    private final Pose partnerPushPoseRed = partnerPushPoseBlue.mirror();
    // Spike 2
    private final Pose travelS2PoseRed = travelS2PoseBlue.mirror();
    private final Pose travelS2Control1Red = travelS2Control1Blue.mirror();
    private final Pose travelS2Control2Red = travelS2Control2Blue.mirror();
    private final Pose pickupS2PoseRed = pickupS2PoseBlue.mirror();
    private final Pose scoreS2PoseRed = scoreS2PoseBlue.mirror();
    private final Pose scoreS2Control1Red = scoreS2Control1Blue.mirror();
    // Open Gate + Grab
    private final Pose openGatePoseRed = openGatePoseBlue.mirror();
    private final Pose openGateControl1Red = openGateControl1Blue.mirror();
    private final Pose openGateControl2Red = openGateControl2Blue.mirror();
    //    private final Pose pickupGatePoseRed = pickupGatePoseBlue.mirror();
    private final Pose scoreGatePoseRed = scoreGatePoseBlue.mirror();
    private final Pose scoreGateControl1Red = scoreGateControl1Blue.mirror();
    private final Pose scoreGateControl2Red = scoreGateControl2Blue.mirror();
    // Spike 3
    private final Pose travelS3PoseRed = travelS3PoseBlue.mirror();
    private final Pose travelS3Control1Red = travelS3Control1Blue.mirror();
    private final Pose pickupS3PoseRed = pickupS3PoseBlue.mirror();
    private final Pose scoreS3PoseRed = scoreS3PoseBlue.mirror();
    private final Pose scoreS3Control1Red = scoreS3Control1Blue.mirror();
    // HP Station
    private final Pose travelHPPoseRed = travelHPPoseBlue.mirror();
    private final Pose travelHPControl1Red = travelHPControl1Blue.mirror();
    private final Pose travelHPControl2Red = travelHPControl2Blue.mirror();
    private final Pose pickupHPPoseRed = pickupHPPoseBlue.mirror();
    private final Pose scoreHPPoseRed = scoreHPPoseBlue.mirror();
    private final Pose scoreHPControl1Red = scoreHPControl1Blue.mirror();
    private final Pose scoreHPControl2Red = scoreHPControl2Blue.mirror();
    // Spike 1
    private final Pose pickupS1PoseRed = pickupS1PoseBlue.mirror();
    private final Pose pickupS1Control1Red = pickupS1Control1Blue.mirror();
    private final Pose scoreS1PoseRed = scoreS1PoseBlue.mirror();
    private final Pose scoreS1Control1Red = scoreS1Control1Blue.mirror();

    private PathChain pushPartnerBlue, travelS2Blue, pickupS2Blue, scoreS2Blue;
    private PathChain openGateBlue, pickupGateBlue, scoreGateBlue;
    private PathChain travelS3Blue, pickupS3Blue, scoreS3Blue;
    private PathChain travelHPBlue, pickupHPBlue, scoreHPBlue;
    private PathChain pickupS1Blue, scoreS1Blue;
    private PathChain pushPartnerRed, travelS2Red, pickupS2Red, scoreS2Red;
    private PathChain openGateRed, pickupGateRed, scoreGateRed;
    private PathChain travelS3Red, pickupS3Red, scoreS3Red;
    private PathChain travelHPRed, pickupHPRed, scoreHPRed;
    private PathChain pickupS1Red, scoreS1Red;

    public void buildPaths() {
        /// BLUE SIDE
        pushPartnerBlue = follower.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, partnerPushPoseBlue)).setLinearHeadingInterpolation(startPoseBlue.getHeading(), partnerPushPoseBlue.getHeading())
                .build();

        travelS2Blue = follower.pathBuilder()
                .addPath(new BezierCurve(partnerPushPoseBlue, travelS2Control1Blue, travelS2Control2Blue, travelS2PoseBlue))
                .setTangentHeadingInterpolation()
                .build();

        pickupS2Blue = follower.pathBuilder()
                .addPath(new BezierLine(travelS2PoseBlue, pickupS2PoseBlue)).setLinearHeadingInterpolation(travelS2PoseBlue.getHeading(), pickupS2PoseBlue.getHeading())
                .build();

        scoreS2Blue = follower.pathBuilder()
                .addPath(new BezierCurve(pickupS2PoseBlue, scoreS2Control1Blue, scoreS2PoseBlue))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

        openGateBlue = follower.pathBuilder()
                .addPath(new BezierCurve(scoreS2PoseBlue, openGateControl1Blue, openGateControl2Blue, openGatePoseBlue))
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

//        pickupGateBlue = follower.pathBuilder()
//                .addPath(new BezierLine(openGatePoseBlue, pickupGatePoseBlue)).setLinearHeadingInterpolation(openGatePoseBlue.getHeading(), pickupGatePoseBlue.getHeading())
//                .build();

        scoreGateBlue = follower.pathBuilder()
                .addPath(new BezierCurve(openGatePoseBlue, scoreGateControl1Blue, scoreGateControl2Blue, scoreGatePoseBlue))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

        travelS3Blue = follower.pathBuilder()
                .addPath(new BezierCurve(scoreGatePoseBlue, travelS3Control1Blue, travelS3PoseBlue))
                .setTangentHeadingInterpolation()
                .build();

        pickupS3Blue = follower.pathBuilder()
                .addPath(new BezierLine(travelS3PoseBlue, pickupS3PoseBlue)).setLinearHeadingInterpolation(travelS3PoseBlue.getHeading(), pickupS3PoseBlue.getHeading())
                .build();

        scoreS3Blue = follower.pathBuilder()
                .addPath(new BezierCurve(pickupS3PoseBlue, scoreS3Control1Blue, scoreS3PoseBlue))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

        travelHPBlue = follower.pathBuilder()
                .addPath(new BezierCurve(scoreS3PoseBlue, travelHPControl1Blue, travelHPControl2Blue, travelHPPoseBlue))
                .setTangentHeadingInterpolation()
                .build();

        pickupHPBlue = follower.pathBuilder()
                .addPath(new BezierLine(travelHPPoseBlue, pickupHPPoseBlue)).setLinearHeadingInterpolation(travelHPPoseBlue.getHeading(), pickupHPPoseBlue.getHeading())
                .build();

        scoreHPBlue = follower.pathBuilder()
                .addPath(new BezierCurve(pickupHPPoseBlue, scoreHPControl1Blue, scoreHPControl2Blue, scoreHPPoseBlue))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(1)
                .build();

        pickupS1Blue = follower.pathBuilder()
                .addPath(new BezierCurve(scoreHPPoseBlue, pickupS1Control1Blue, pickupS1PoseBlue))
                .setTangentHeadingInterpolation()
                .build();

        scoreS1Blue = follower.pathBuilder()
                .addPath(new BezierCurve(pickupS1PoseBlue, scoreS1Control1Blue, scoreS1PoseBlue))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

        /// RED SIDE
        /// RED SIDE
        pushPartnerRed = follower.pathBuilder()
                .addPath(new BezierLine(startPoseRed, partnerPushPoseRed))
                .setLinearHeadingInterpolation(startPoseRed.getHeading(), partnerPushPoseRed.getHeading())
                .build();

        travelS2Red = follower.pathBuilder()
                .addPath(new BezierCurve(partnerPushPoseRed, travelS2Control1Red, travelS2Control2Red, travelS2PoseRed))
                .setTangentHeadingInterpolation()
                .build();

        pickupS2Red = follower.pathBuilder()
                .addPath(new BezierLine(travelS2PoseRed, pickupS2PoseRed))
                .setLinearHeadingInterpolation(travelS2PoseRed.getHeading(), pickupS2PoseRed.getHeading())
                .build();

        scoreS2Red = follower.pathBuilder()
                .addPath(new BezierCurve(pickupS2PoseRed, scoreS2Control1Red, scoreS2PoseRed))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

        openGateRed = follower.pathBuilder()
                .addPath(new BezierCurve(scoreS2PoseRed, openGateControl1Red, openGateControl2Red, openGatePoseRed))
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

//        pickupGateRed = follower.pathBuilder()
//                .addPath(new BezierLine(openGatePoseRed, pickupGatePoseRed)).setLinearHeadingInterpolation(openGatePoseRed.getHeading(), pickupGatePoseRed.getHeading())
//                .build();

        scoreGateRed = follower.pathBuilder()
                .addPath(new BezierCurve(openGatePoseBlue, scoreGateControl1Red, scoreGateControl2Red, scoreGatePoseRed))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

        travelS3Red = follower.pathBuilder()
                .addPath(new BezierCurve(scoreGatePoseRed, travelS3Control1Red, travelS3PoseRed))
                .setTangentHeadingInterpolation()
                .build();

        pickupS3Red = follower.pathBuilder()
                .addPath(new BezierLine(travelS3PoseRed, pickupS3PoseRed)).setLinearHeadingInterpolation(travelS3PoseRed.getHeading(), pickupS3PoseRed.getHeading())
                .build();

        scoreS3Red = follower.pathBuilder()
                .addPath(new BezierCurve(pickupS3PoseRed, scoreS3Control1Red, scoreS3PoseRed))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

        travelHPRed = follower.pathBuilder()
                .addPath(new BezierCurve(scoreS3PoseRed, travelHPControl1Red, travelHPControl2Red, travelHPPoseRed))
                .setTangentHeadingInterpolation()
                .build();

        pickupHPRed = follower.pathBuilder()
                .addPath(new BezierLine(travelHPPoseRed, pickupHPPoseRed)).setLinearHeadingInterpolation(travelHPPoseRed.getHeading(), pickupHPPoseRed.getHeading())
                .build();

        scoreHPRed = follower.pathBuilder()
                .addPath(new BezierCurve(pickupHPPoseRed, scoreHPControl1Red, scoreHPControl2Red, scoreHPPoseRed))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(1)
                .build();

        pickupS1Red = follower.pathBuilder()
                .addPath(new BezierCurve(scoreHPPoseRed, pickupS1Control1Red, pickupS1PoseRed))
                .setTangentHeadingInterpolation()
                .build();

        scoreS1Red = follower.pathBuilder()
                .addPath(new BezierCurve(pickupS1PoseRed, scoreS1Control1Red, scoreS1PoseRed))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            ///  SPIN UP + PUSH PARTNER
            case 0:
                cyclingFarZone = true;
                runShooter = true;
                runTurret = true;
                follower.followPath(blueAlliance ? pushPartnerBlue : pushPartnerRed);
                setPathState(1);
                break;
            /// SHOOT PRELOAD
            case 1:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
                    rapidFireAction();
                    setPathState(2);
                }
                break;
            /// TRAVEL S2
            case 2:
                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    cyclingFarZone = false;
                    runShooter = false;
                    runTurret = false;
//                    follower.followPath(blueAlliance ? pushPartnerBlue : pushPartnerRed);
                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
                    follower.followPath(blueAlliance ? travelS2Blue : travelS2Red, true);
                    setPathState(4);
//                    setPathState(3);
                }
                break;
            /// PICK UP S2
            case 4:
                if (follower.atPose(blueAlliance ? travelS2PoseBlue : travelS2PoseRed, AT_POSE_X_TOLERANCE, AT_POSE_Y_TOLERANCE)) {
                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
                    follower.followPath(blueAlliance ? pickupS2Blue : pickupS2Red, SLOW_PATH_MAX_POWER, true);
                    setPathState(5);
                }
                break;
            /// SCORE S2
            case 5:
                if (follower.atPose(blueAlliance ? pickupS2PoseBlue : pickupS2PoseRed, AT_POSE_X_TOLERANCE, AT_POSE_Y_TOLERANCE)) {
                    delayedIdle();
                    cyclingFarZone = false;
                    runShooter = true;
                    runTurret = true;
                    follower.followPath(blueAlliance ? scoreS2Blue : scoreS2Red, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()
                        && shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
                    rapidFireAction();
                    setPathState(7);
                }
                break;
            /// OPEN GATE
            case 7:
                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    cyclingFarZone = true;
                    runShooter = false;
                    runTurret = false;
                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
                    follower.followPath(blueAlliance ? openGateBlue : openGateRed, 0.75, true);
                    setPathState(9);
                }
                break;
            /// PICK UP FROM GATE
//            case 8:
//                if (!follower.isBusy()) {
//                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
//                    follower.followPath(blueAlliance ? pickupGateBlue : pickupGateRed, true);
//                    setPathState(9);
//                }
//                break;
            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > WAIT_GATE_DELAY) {
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.intakeFullPower()),
                            new SleepAction(0.5),
                            new InstantAction(() -> intake.idle())
                    ));                    follower.followPath(blueAlliance ? scoreGateBlue : scoreGateRed, true);
                    setPathState(10);
                }
                break;
            /// SCORE GATE
            case 10:
                if (pathTimer.getElapsedTime() > 1200) {
                    cyclingFarZone = true;
                    runShooter = true;
                    runTurret = true;
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()
                        && shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
                    rapidFireAction();
                    setPathState(12);
                }
                break;
            /// TRAVEL S3
            case 12:
                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runningActions.add(new InstantAction(() -> intake.intake()));
                    follower.followPath(blueAlliance ? travelS3Blue : travelS3Red, true);
                    setPathState(13);
                }
                break;
            /// PICK UP S3
            case 13:
                if (follower.atPose(blueAlliance ?  travelS3PoseBlue: travelS3PoseRed, AT_POSE_X_TOLERANCE, AT_POSE_Y_TOLERANCE)) {
                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
                    follower.followPath(blueAlliance ? pickupS3Blue : pickupS3Red, SLOW_PATH_MAX_POWER, true);
                    setPathState(14);
                }
                break;
            /// SCORE S3
            case 14:
                if (follower.atPose(blueAlliance ? pickupS3PoseBlue : pickupS3PoseRed, AT_POSE_X_TOLERANCE, AT_POSE_Y_TOLERANCE)) {
                    delayedIdle();
                    cyclingFarZone = true;
                    runShooter = true;
                    runTurret = true;
                    follower.followPath(blueAlliance ? scoreS3Blue : scoreS3Red, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()
                        && shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
                    rapidFireAction();
                    setPathState(16);
                }
                break;
            /// TRAVEL HP
            case 16:
                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    cyclingFarZone = false;
                    runShooter = false;
                    runTurret = false;
                    runningActions.add(new InstantAction(() -> intake.intake()));
                    follower.followPath(blueAlliance ? travelHPBlue : travelHPRed, true);
                    setPathState(17);
                }
                break;
            /// PICK UP HP
            case 17:
                if (follower.atPose(blueAlliance ? travelHPPoseBlue: travelHPPoseRed, AT_POSE_X_TOLERANCE, AT_POSE_Y_TOLERANCE)) {
                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
                    follower.followPath(blueAlliance ?  pickupHPBlue: pickupHPRed, 0.4, true);
                    setPathState(18);
                }
                break;
            /// SCORE HP
            case 18:
                if (follower.atPose(blueAlliance ? pickupHPPoseBlue : pickupHPPoseRed, AT_POSE_X_TOLERANCE, AT_POSE_Y_TOLERANCE) || pathTimer.getElapsedTime() > 1200) {
                    delayedIdle();
                    cyclingFarZone = false; // redundant
                    follower.followPath(blueAlliance ? scoreHPBlue : scoreHPRed, true);
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTimer.getElapsedTime() > 2200) {
                    runShooter = true;
                    runTurret = true;
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()
                        && shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
                    rapidFireAction();
                    setPathState(21);
                }
                break;
            /// PICK UP S1
            case 21:
                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    cyclingFarZone = false;
                    runShooter = false;
                    runTurret = false;
                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
                    follower.followPath(blueAlliance ? pickupS1Blue : pickupS1Red, SLOW_PATH_MAX_POWER, true);
                    setPathState(22);
                }
                break;
            /// SCORE S1
            case 22:
                if (follower.atPose(blueAlliance ? pickupS1PoseBlue: pickupS1PoseRed, AT_POSE_X_TOLERANCE, AT_POSE_Y_TOLERANCE)) {
                    delayedIdle();
                    cyclingFarZone = false; // redundant
                    runShooter = true;
                    runTurret = true;
                    follower.followPath(blueAlliance ? scoreS1Blue: scoreS1Red, true);
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy()
                        && shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
                    rapidFireAction();
                    setPathState(24);
                }
                break;
            /// SHUT DOWN
            case 24: // park
                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false; // back to idle, will turn off when opmode stops
                    runTurret = false; // return to center
                    runningActions.add(new SequentialAction( // technically redundant, but doesn't hurt to have
                            new InstantAction(() -> intake.idle()),
                            new InstantAction(() -> intake.stowBallPusher()),
                            new InstantAction(() -> shooter.closeLatch())
                    ));
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        robotHardware.initialize(this);
        intake.initialize(this, robotHardware);
        llVision.initialize(this, robotHardware);
        shooter.initialize(this, robotHardware, llVision);
        turret.initialize(this, robotHardware, llVision, true);
        lights.initialize(this, robotHardware);
        brake.initialize(this, robotHardware);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        // set servos for auto
        intake.stowBallPusher();
        shooter.closeLatch();
        lights.setColor(blueAlliance ? RGBLights.Colors.BLUE : RGBLights.Colors.RED);
        brake.stowBrake();
    }

    @Override
    public void init_loop() {
        if (gamepad1.b || gamepad2.b) {
            blueAlliance = false;
            lights.setColor(RGBLights.Colors.RED);
        } else if (gamepad1.x || gamepad2.x) {
            blueAlliance = true;
            lights.setColor(RGBLights.Colors.BLUE);
        }

        follower.update();

        telemetry.addLine("B for RED | X for BLUE");
        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
        telemetry.addData("x ", follower.getPose().getX());
        telemetry.addData("y ", follower.getPose().getY());
        telemetry.addData("heading ", Math.toDegrees(follower.getPose().getHeading()));
        if (Math.abs(follower.getVelocity().getMagnitude()) > 1 || Math.abs(follower.getAngularVelocity()) > 0.2) {
            telemetry.addLine("PINPOINT IS COOKED");
            telemetry.addLine("PINPOINT IS COOKED");
            telemetry.addLine("PINPOINT IS COOKED");
            telemetry.addLine("PINPOINT IS COOKED");
            telemetry.addLine("PINPOINT IS COOKED");
        }
        telemetry.addData("velocity magnitude", Math.abs(follower.getVelocity().getMagnitude()));
        telemetry.addData("angular velocity", Math.abs(follower.getAngularVelocity()));
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setStartingPose(blueAlliance ? startPoseBlue : startPoseRed);
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) { hub.clearBulkCache(); }

        // RR Actions
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) { if (action.run(packet)) { newActions.add(action); } }
        runningActions = newActions;

        follower.update();
        Pose currentPose = follower.getPose();

        // loops
        intake.operateCurrentLimiting();
        shooter.operateOdomAuto(currentPose.getX(), currentPose.getY(), blueAlliance, runShooter, cyclingFarZone);
        turret.operateOdomAuto(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()), blueAlliance, runTurret);
        lights.operateAuto(); // rainbow strobe (can change endpoint colors and speed)
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addLine("STATE TELEMETRY");
        telemetry.addData("path state ", pathState);
        telemetry.addData("runShooter? ", runShooter);
        telemetry.addData("runTurret? ", runTurret);
        telemetry.addLine("\n");
        telemetry.addLine("PEDRO TELEMETRY");
        telemetry.addData("isBusy? ", follower.isBusy());
        telemetry.addData("isRobotStuck? ", follower.isRobotStuck());
        telemetry.addData("x ", currentPose.getX());
        telemetry.addData("y ", currentPose.getY());
        telemetry.addData("heading ", Math.toDegrees(currentPose.getHeading()));
        telemetry.addLine("\n");
        telemetry.addData("looptimes ", elapsedtime.milliseconds());
        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");

        elapsedtime.reset();
    }

    @Override
    public void stop() {
        Globals.autoEndPose = follower.getPose();
        Globals.blueAlliance = blueAlliance;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // method for rapid firing to reduce
    private void rapidFireAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> shooter.openLatch()),
                new InstantAction(() -> intake.intakeFullPower()),
                new SleepAction(RAPID_FIRE_DELAY),
                new InstantAction(() -> intake.deployBallPusher()),
                new SleepAction(PUSHER_DELAY),
                new InstantAction(() -> pathTimer.resetTimer()), // kinda weird, but necessary so robot waits for a moment after scoring
                new InstantAction(() -> intake.stowBallPusher()),
                new InstantAction(() -> shooter.closeLatch()),
                new InstantAction(() -> intake.idle())
        ));
    }

    private void delayedIdle() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> intake.intakeFullPower()),
                new SleepAction(1.25),
                new InstantAction(() -> intake.idle())
        ));
    }
}
