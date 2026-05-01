package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Util.LimelightVision;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Turret;
import org.firstinspires.ftc.teamcode.Util.Globals;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Util.RGBLights;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "21 ball NO S3", group = "A", preselectTeleOp = "Full Teleop DUAL DRIVER")
public class TwentyOneBallNearNOS3 extends OpMode {
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private List<LynxModule> allHubs;
    private ElapsedTime elapsedtime;
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3

    private RobotHardware robotHardware = new RobotHardware();
    private Intake intake = new Intake();
    private Turret turret = new Turret();
    private Shooter shooter = new Shooter();
    private LimelightVision llVision = new LimelightVision();
    private RGBLights lights = new RGBLights();

    ///  CONSTANTS
    private boolean blueAlliance = true;
    private volatile boolean runShooter = false;
    private volatile boolean runTurret = false;
    private volatile boolean cyclingFarZone = false;
    private volatile boolean currentlyShooting = false;
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    private volatile boolean extraCycleComplete = false;
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    private final double TRANSFER_ONLY_DELAY = 0.03;
    private final double RAPID_FIRE_DELAY = 0.4;
    private final double WAIT_GATE_DELAY = 1.6; // TODO
    private final double GATE_BRAKING_START = 1.3; // idk how big of a difference this makes
    private final double DELAY_BEFORE_MOVING = 50; // milliseconds

    /// BLUE SIDE POSES
    /// blue start 20 113.8 180
    /// 20 114.3 180 (backup)

    /// red start 124.8 114 0
    /// 125 114.2 0 (backup)

    /// blue gate 15.7 59.3 140
    /// red gate  130.5 60.8 40
    private final Pose startPoseBlue    = new Pose(20, 113.8, Math.toRadians(180)); // TODO measure accurately & test with 72,72 opmode
    private final Pose score123PoseBlue = new Pose(64,69, Math.toRadians(180));
    private final Pose prepGrab456PoseBlue = new Pose(40.8, 60, Math.toRadians(180));
    private final Pose grab456PoseBlue  = new Pose(19,60, Math.toRadians(180));
    private final Pose score456PoseBlue = new Pose(60.5,73.5, Math.toRadians(170));
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    private final Pose grab789PoseBlue  = /** BLUE BLUE */new Pose(14.9, 59.45, Math.toRadians(148)); /// gate
    ///                                                               15.7
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    private final Pose grab789PoseRed   = /** RED RED */new Pose(131.5, 59.45, Math.toRadians(32)); /// gate
    ///                                                             130.7

    private final Pose score789PoseBlue = new Pose(61.2,70.6, Math.toRadians(170));
    private final Pose grab101112PoseBlue  = grab789PoseBlue;
    private final Pose score101112PoseBlue = score789PoseBlue;
    private final Pose grab131415PoseBlue  = grab789PoseBlue;
    private final Pose score131415PoseBlue = score789PoseBlue;
    private final Pose prepGrab192021PoseBlue = new Pose(46, 84.5, Math.toRadians(180));
    private final Pose grab192021PoseBlue  = new Pose(19,84.5, Math.toRadians(180));
    private final Pose score192021PoseBlue = new Pose(49,84.5, Math.toRadians(180));
    private final Pose parkPoseBlue        = new Pose(47,83.5, Math.toRadians(225));
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3

    /// RED SIDE POSES
    private final Pose startPoseRed     = new Pose(124.3, 114, Math.toRadians(0));
    private final Pose score123PoseRed  = score123PoseBlue.mirror();
    private final Pose prepGrab456PoseRed = prepGrab456PoseBlue.mirror();
    private final Pose grab456PoseRed   = grab456PoseBlue.mirror();
    private final Pose score456PoseRed  = score456PoseBlue.mirror();


    private final Pose score789PoseRed  = score789PoseBlue.mirror();
    private final Pose grab101112PoseRed  = grab789PoseRed;
    private final Pose score101112PoseRed = score789PoseRed;
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    private final Pose grab131415PoseRed  = grab789PoseRed;
    private final Pose score131415PoseRed = score789PoseRed;
    private final Pose prepGrab192021PoseRed = prepGrab192021PoseBlue.mirror();
    private final Pose grab192021PoseRed  = grab192021PoseBlue.mirror();
    private final Pose score192021PoseRed = score192021PoseBlue.mirror();
    private final Pose parkPoseRed        = parkPoseBlue.mirror();

    // PathChains
    private PathChain BScore123, BGrab456, BScore456, BGrab789, BScore789;
    private PathChain BGrab101112, BScore101112, BGrab131415, BScore131415;
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    ///  NO S3  NO S3  NO S3  NO S3  NO S3
    private PathChain BGrab192021, BScore192021, BPark;
    private PathChain RScore123, RGrab456, RScore456, RGrab789, RScore789;
    private PathChain RGrab101112, RScore101112, RGrab131415, RScore131415;
    private PathChain RGrab192021, RScore192021, RPark;

    public void buildPaths() {
        /// BLUE SIDE
        BScore123 = follower.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, score123PoseBlue))
                .setLinearHeadingInterpolation(startPoseBlue.getHeading(), score123PoseBlue.getHeading())
//                .setGlobalDeceleration()
                .build();
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

        BGrab456 = follower.pathBuilder()
                .addPath(new BezierLine(score123PoseBlue, prepGrab456PoseBlue))
                .setLinearHeadingInterpolation(score123PoseBlue.getHeading(), prepGrab456PoseBlue.getHeading())
                .addPath(new BezierLine(prepGrab456PoseBlue, grab456PoseBlue))
                .setLinearHeadingInterpolation(prepGrab456PoseBlue.getHeading(), grab456PoseBlue.getHeading())
                .build();

        BScore456 = follower.pathBuilder()
                .addPath(new BezierLine(grab456PoseBlue, score456PoseBlue))
                .setLinearHeadingInterpolation(grab456PoseBlue.getHeading(), score456PoseBlue.getHeading())
                .build();
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

        BGrab789 = follower.pathBuilder()
                .addPath(new BezierLine(score456PoseBlue, grab789PoseBlue))
                .setLinearHeadingInterpolation(score456PoseBlue.getHeading(), grab789PoseBlue.getHeading())
                .setGlobalDeceleration(GATE_BRAKING_START)
                .build();
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        BScore789 = follower.pathBuilder()
                .addPath(new BezierLine(grab789PoseBlue, score789PoseBlue))
                .setLinearHeadingInterpolation(grab789PoseBlue.getHeading(), score789PoseBlue.getHeading())
                .build();

        BGrab101112 = follower.pathBuilder()
                .addPath(new BezierLine(score789PoseBlue, grab101112PoseBlue))
                .setLinearHeadingInterpolation(score789PoseBlue.getHeading(), grab101112PoseBlue.getHeading())
                .setGlobalDeceleration(GATE_BRAKING_START)
                .build();
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

        BScore101112 = follower.pathBuilder()
                .addPath(new BezierLine(grab101112PoseBlue, score101112PoseBlue))
                .setLinearHeadingInterpolation(grab101112PoseBlue.getHeading(), score101112PoseBlue.getHeading())
                .build();

        BGrab131415 = follower.pathBuilder()
                .addPath(new BezierLine(score101112PoseBlue, grab131415PoseBlue))
                .setLinearHeadingInterpolation(score101112PoseBlue.getHeading(), grab131415PoseBlue.getHeading())
                .setGlobalDeceleration(GATE_BRAKING_START)
                .build();

        BScore131415 = follower.pathBuilder()
                .addPath(new BezierLine(grab131415PoseBlue, score131415PoseBlue))
                .setLinearHeadingInterpolation(grab131415PoseBlue.getHeading(), score131415PoseBlue.getHeading())
                .build();
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

        BGrab192021 = follower.pathBuilder()
                .addPath(new BezierLine(score131415PoseBlue, prepGrab192021PoseBlue))
                .setLinearHeadingInterpolation(score131415PoseBlue.getHeading(), prepGrab192021PoseBlue.getHeading())
                .addPath(new BezierLine(prepGrab192021PoseBlue, grab192021PoseBlue))
                .setLinearHeadingInterpolation(prepGrab192021PoseBlue.getHeading(), grab192021PoseBlue.getHeading())
                .build();

        BScore192021 = follower.pathBuilder()
                .addPath(new BezierLine(grab192021PoseBlue, score192021PoseBlue))
                .setLinearHeadingInterpolation(grab192021PoseBlue.getHeading(), score192021PoseBlue.getHeading())
                .build();
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

        BPark = follower.pathBuilder()
                .addPath(new BezierLine(score192021PoseBlue, parkPoseBlue))
                .setLinearHeadingInterpolation(score192021PoseBlue.getHeading(), parkPoseBlue.getHeading())
                .build();

        /// RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE
        /// RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE
        /// RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE

        RScore123 = follower.pathBuilder()
                .addPath(new BezierLine(startPoseRed, score123PoseRed))
                .setLinearHeadingInterpolation(startPoseRed.getHeading(), score123PoseRed.getHeading())
                .build();
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

        RGrab456 = follower.pathBuilder()
                .addPath(new BezierLine(score123PoseRed, prepGrab456PoseRed))
                .setLinearHeadingInterpolation(score123PoseRed.getHeading(), prepGrab456PoseRed.getHeading())
                .addPath(new BezierLine(prepGrab456PoseRed, grab456PoseRed))
                .setLinearHeadingInterpolation(prepGrab456PoseRed.getHeading(), grab456PoseRed.getHeading())
                .build();

        RScore456 = follower.pathBuilder()
                .addPath(new BezierLine(grab456PoseRed, score456PoseRed))
                .setLinearHeadingInterpolation(grab456PoseRed.getHeading(), score456PoseRed.getHeading())
                .build();
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

        RGrab789 = follower.pathBuilder()
                .addPath(new BezierLine(score456PoseRed, grab789PoseRed))
                .setLinearHeadingInterpolation(score456PoseRed.getHeading(), grab789PoseRed.getHeading())
                .setGlobalDeceleration(GATE_BRAKING_START)
                .build();

        RScore789 = follower.pathBuilder()
                .addPath(new BezierLine(grab789PoseRed, score789PoseRed))
                .setLinearHeadingInterpolation(grab789PoseRed.getHeading(), score789PoseRed.getHeading())
                .build();

        RGrab101112 = follower.pathBuilder()
                .addPath(new BezierLine(score789PoseRed, grab101112PoseRed))
                .setLinearHeadingInterpolation(score789PoseRed.getHeading(), grab101112PoseRed.getHeading())
                .setGlobalDeceleration(GATE_BRAKING_START)
                .build();
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

        RScore101112 = follower.pathBuilder()
                .addPath(new BezierLine(grab101112PoseRed, score101112PoseRed))
                .setLinearHeadingInterpolation(grab101112PoseRed.getHeading(), score101112PoseRed.getHeading())
                .build();

        RGrab131415 = follower.pathBuilder()
                .addPath(new BezierLine(score101112PoseRed, grab131415PoseRed))
                .setLinearHeadingInterpolation(score101112PoseRed.getHeading(), grab131415PoseRed.getHeading())
                .setGlobalDeceleration(GATE_BRAKING_START)
                .build();

        RScore131415 = follower.pathBuilder()
                .addPath(new BezierLine(grab131415PoseRed, score131415PoseRed))
                .setLinearHeadingInterpolation(grab131415PoseRed.getHeading(), score131415PoseRed.getHeading())
                .build();
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        RGrab192021 = follower.pathBuilder()
                .addPath(new BezierLine(score131415PoseRed, prepGrab192021PoseRed))
                .setLinearHeadingInterpolation(score131415PoseRed.getHeading(), prepGrab192021PoseRed.getHeading())
                .addPath(new BezierLine(prepGrab192021PoseRed, grab192021PoseRed))
                .setLinearHeadingInterpolation(prepGrab192021PoseRed.getHeading(), grab192021PoseRed.getHeading())
                .build();

        RScore192021 = follower.pathBuilder()
                .addPath(new BezierLine(grab192021PoseRed, score192021PoseRed))
                .setLinearHeadingInterpolation(grab192021PoseRed.getHeading(), score192021PoseRed.getHeading())
                .build();

        RPark = follower.pathBuilder()
                .addPath(new BezierLine(score192021PoseRed, parkPoseRed))
                .setLinearHeadingInterpolation(score192021PoseRed.getHeading(), parkPoseRed.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            ///  SPIN UP + DRIVE
            case 0:
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                Globals.blueGoalX = 3;
                Globals.blueGoalY = 136;
                Globals.redGoalX = Globals.PERMANENT_redGoalX;
                Globals.redGoalY = Globals.PERMANENT_redGoalY;
                cyclingFarZone = false; // redundant
                runShooter = true;
                runTurret = true;
                shooter.openLatch();
                follower.followPath(blueAlliance ? BScore123 : RScore123, true);
                setPathState(1);
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                break;
            /// SCORE PRELOAD (123)
            case 1:
                if (shooter.atTargetRPM && turret.atTargetAngle && pathTimer.getElapsedTimeSeconds() > 1 && shooter.latchOpen) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && !currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    Globals.blueGoalX = Globals.PERMANENT_blueGoalX;
                    Globals.blueGoalY = Globals.PERMANENT_blueGoalY;
                    Globals.redGoalX = Globals.PERMANENT_redGoalX;
                    Globals.redGoalY = Globals.PERMANENT_redGoalY;
                    shooter.closeLatch();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrab456 : RGrab456, 0.8, true);
                    setPathState(3);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            /// GRAB S2 (456)
            case 3:
                if (!follower.isBusy() || intake.isFull) {
                    delayedIdleAction();
                    openLatchAction();
//                    intake.idle();
//                    runShooter = true;
                    follower.followPath(blueAlliance ? BScore456 : RScore456, true);
                    setPathState(4);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            /// SCORE S2 (456)
            case 4:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle && shooter.latchOpen) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(6);
                }
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                break;
            /// GRAB GATE (789)
            case 6:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    intake.deployIntake();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrab789 : RGrab789, true);
                    setPathState(7);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            // OPEN GATE
            case 7:
                if (!follower.isBusy()) {
                    setPathState(8);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            // WAIT UNTIL INTAKE FULL OR TIME LIMIT PASSED
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > WAIT_GATE_DELAY || intake.isFull) {
                    intake.idle();
                    intake.stowIntake();
//                    shooter.openLatch();
                    openLatchAction();
                    runShooter = true;
                    follower.followPath(blueAlliance ? BScore789 : RScore789, true);
                    setPathState(9);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            /// SCORE GATE (789)
            case 9:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle && shooter.latchOpen) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(10);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            /// GRAB GATE (101112)
            case 10:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    intake.deployIntake();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrab101112 : RGrab101112, true);
                    setPathState(11);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            // OPEN GATE
            case 11:
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            // WAIT UNTIL INTAKE FULL OR TIME LIMIT PASSED
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > WAIT_GATE_DELAY || intake.isFull) {
                    intake.idle();
                    intake.stowIntake();
//                    shooter.openLatch();
                    openLatchAction();
                    runShooter = true;
                    follower.followPath(blueAlliance ? BScore101112 : RScore101112, true);
                    setPathState(14);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            /// SCORE GATE (101112)
            case 14:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle && shooter.latchOpen) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(15);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            /// GRAB GATE (131415)
            case 15:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    intake.deployIntake();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrab131415 : RGrab131415, true);
                    setPathState(16);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            // OPEN GATE
            case 16:
                if (!follower.isBusy()) {
                    setPathState(17);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            // WAIT UNTIL INTAKE FULL OR TIME LIMIT PASSED
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > WAIT_GATE_DELAY || intake.isFull) {
                    intake.idle();
                    intake.stowIntake();
//                    shooter.openLatch();
                    openLatchAction();
                    runShooter = true;
                    follower.followPath(blueAlliance ? BScore131415 : RScore131415, true);
                    setPathState(18);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            /// SCORE GATE (131415)
            case 18:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle && shooter.latchOpen) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(23);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            /// GRAB S1 (192021)
            case 23:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    shooter.closeLatch();
                    closeLatchAction();
                    intake.deployIntake();
                    intake.intakingIntake();
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    if (extraCycleComplete) {
                        follower.followPath(blueAlliance ? BGrab192021 : RGrab192021, true);
                        setPathState(24);
                        ///  NO S3  NO S3  NO S3  NO S3  NO S3
                        ///  NO S3  NO S3  NO S3  NO S3  NO S3
                        ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    } else {
                        extraCycleComplete = true;
                        follower.followPath(blueAlliance ? BGrab131415 : RGrab131415, true);
                        setPathState(16);
                        ///  NO S3  NO S3  NO S3  NO S3  NO S3
                        ///  NO S3  NO S3  NO S3  NO S3  NO S3
                        ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    }
                }
                break;
            /// SCORE S1
            case 24:
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                if (!follower.isBusy() || intake.isFull) {
                    delayedIdleAction();
                    openLatchAction();
                    follower.followPath(blueAlliance ? BScore192021 : RScore192021, true);
                    setPathState(25);
                }
                break;
            case 25:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle && shooter.latchOpen) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(26);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            /// PARK
            case 26:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    // park
                    follower.followPath(blueAlliance ? BPark : RPark, true);
                    setPathState(-1);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
            /// SHUT DOWN
            case -1:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false; // back to idle, will turn off when opmode stops
                    runTurret = false; // return to center
                    intake.idle();
                    intake.stowIntake();
                    shooter.closeLatch();
                    setPathState(-6962);
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                    ///  NO S3  NO S3  NO S3  NO S3  NO S3
                }
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        robotHardware.initialize(this);
        intake.initialize(this, robotHardware);
        llVision.initialize(this, robotHardware);
        shooter.initialize(this, robotHardware);
        turret.initialize(this, robotHardware);
        lights.initialize(this, robotHardware);
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        // set servos for auto
        intake.deployIntake();
        shooter.closeLatch();
        shooter.initHood();
        lights.setColor(blueAlliance ? RGBLights.Colors.BLUE : RGBLights.Colors.RED);
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
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
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

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
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        telemetry.addData("velocity magnitude", Math.abs(follower.getVelocity().getMagnitude()));
        telemetry.addData("angular velocity", Math.abs(follower.getAngularVelocity()));
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setStartingPose(blueAlliance ? startPoseBlue : startPoseRed);
        opmodeTimer.resetTimer();
        setPathState(0);
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) { hub.clearBulkCache(); }

        // RR Actions
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) { if (action.run(packet)) { newActions.add(action); } }
        runningActions = newActions;
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

        follower.update();
        Pose currentPose = follower.getPose();

        // loops
        lights.operateAuto(); // rainbow strobe (can change endpoint colors and speed)
        autonomousPathUpdate();

        intake.operateAuto(currentlyShooting);
        shootWhileMoveCalcsSimple(currentPose);
//        shooter.operateAuto(currentPose.getX(), currentPose.getY(), blueAlliance, runShooter, cyclingFarZone);
//        turret.operateAuto(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()), blueAlliance, runTurret);


        // Feedback to Driver Hub for debugging
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        telemetry.addLine("STATE TELEMETRY");
        telemetry.addData("path state ", pathState);
        telemetry.addData("runShooter? ", runShooter);
        telemetry.addData("runTurret? ", runTurret);
        telemetry.addLine("\n");
        telemetry.addLine("PEDRO TELEMETRY");
        telemetry.addData("isBusy? ", follower.isBusy());
        telemetry.addData("isRobotStuck? ", follower.isRobotStuck());
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        telemetry.addData("x ", currentPose.getX());
        telemetry.addData("y ", currentPose.getY());
        telemetry.addData("heading ", Math.toDegrees(currentPose.getHeading()));
        telemetry.addLine("\n");
        telemetry.addData("looptimes ", elapsedtime.milliseconds());
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");

        elapsedtime.reset();
    }

    @Override
    public void stop() {
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        Globals.autoEndPose = follower.getPose();
        Globals.blueAlliance = blueAlliance;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // method for rapid firing to reduce
    private void rapidFireAction() {
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        runningActions.add(new SequentialAction(
                new InstantAction(() -> intake.runTransferOnly()),
                new SleepAction(TRANSFER_ONLY_DELAY),
                new InstantAction(() -> intake.shootingIntake(cyclingFarZone)),
                new SleepAction(RAPID_FIRE_DELAY),
                new InstantAction(() -> intake.idle()),
                new InstantAction(() -> shooter.closeLatch()),
                new InstantAction(() -> currentlyShooting = false)
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ));
    }

    private void delayedIdleAction() {
        runningActions.add(new SequentialAction(
                new SleepAction(0.3),
                new InstantAction(() -> intake.idle())
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
                ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ));
    }

    private void delayedIdleAction(double customDelay) {
        runningActions.add(new SequentialAction(
                new SleepAction(customDelay),
                new InstantAction(() -> intake.idle())
        ));
    }

    private void openLatchAction() {
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        runningActions.add(new SequentialAction(
                new SleepAction(0.3),
                new InstantAction(() -> shooter.openLatch())
        ));
    }

    private void openLatchAction(double customDelay) {
        runningActions.add(new SequentialAction(
                new SleepAction(customDelay),
                new InstantAction(() -> shooter.openLatch())
        ));
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
    }

    private void closeLatchAction() {
        runningActions.add(new SequentialAction(
                new SleepAction(0.1),
                new InstantAction(() -> shooter.closeLatch()),
                new SleepAction(0.1),
                new InstantAction(() -> shooter.closeLatch())
        ));    }

    private void shootWhileMoveCalcsSimple(Pose currentPose) { // short for calculator
        double temp_time = elapsedtime.milliseconds();
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

        double xVel = follower.getVelocity().getXComponent();
        double yVel = follower.getVelocity().getYComponent();
        double headingDegrees = Math.toDegrees(currentPose.getHeading());
        double currentXDist = (blueAlliance ? Globals.blueGoalX : Globals.redGoalX) - currentPose.getX();
        double currentYDist = (blueAlliance ? Globals.blueGoalY : Globals.redGoalY) - currentPose.getY();

        double currentDist = Math.hypot(currentXDist, currentYDist);
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

        telemetry.addLine("\n=== SHOOT WHILE MOVE CALCS ===");

        if (Math.abs(xVel) < 0.1 && Math.abs(xVel) < 0.1) {
            shooter.operateSWMAuto(currentXDist, currentYDist, runShooter, cyclingFarZone);
            turret.operateSWMAuto(currentXDist, currentYDist, headingDegrees, runTurret);

            telemetry.addLine("0");
            telemetry.addLine("0");
            telemetry.addLine("0");
        }
        else {
            ///  NO S3  NO S3  NO S3  NO S3  NO S3
            ///  NO S3  NO S3  NO S3  NO S3  NO S3
            ///  NO S3  NO S3  NO S3  NO S3  NO S3
            double airtime = -0.000012 * Math.pow(currentDist, 2) + 0.0036 * currentDist + 0.43;
            double adjustedXDist = currentXDist - (xVel * airtime);
            double adjustedYDist = currentYDist - (yVel * airtime);

            shooter.operateSWMAuto(adjustedXDist, adjustedYDist, runShooter, cyclingFarZone);
            turret.operateSWMAuto(adjustedXDist, adjustedYDist, headingDegrees, runTurret);

            telemetry.addData("airtime", airtime);
            telemetry.addData("adjustedXDist", adjustedXDist);
            telemetry.addData("adjustedYDist", adjustedYDist);
        }
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3
        ///  NO S3  NO S3  NO S3  NO S3  NO S3

        telemetry.addData("xVel", xVel);
        telemetry.addData("yVel", yVel);
        telemetry.addData("processing time taken", elapsedtime.milliseconds() - temp_time);
    }
}
