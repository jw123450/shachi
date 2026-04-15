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

@Autonomous(name = "21 ball", group = "A", preselectTeleOp = "Full Teleop DUAL DRIVER")
public class TwentyOneBallNear extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private List<LynxModule> allHubs;
    private ElapsedTime elapsedtime;


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
    private final double TRANSFER_ONLY_DELAY = 0.03;
    private final double RAPID_FIRE_DELAY = 0.4;
    private final double WAIT_GATE_DELAY = 1.5; // TODO

    private final double DELAY_BEFORE_MOVING = 50; // milliseconds

    /// BLUE SIDE POSES
    private final Pose startPoseBlue    = new Pose(20,115, Math.toRadians(143)); // TODO measure accurately & test with 72,72 opmode
    private final Pose score123PoseBlue = new Pose(64,69, Math.toRadians(180));
    private final Pose prepGrab456PoseBlue = new Pose(40.8, 60, Math.toRadians(180));
    private final Pose grab456PoseBlue  = new Pose(17,60, Math.toRadians(180));
    private final Pose score456PoseBlue = new Pose(60.5,73.5, Math.toRadians(170));
    private final Pose grab789PoseBlue  = new Pose(15,62.45, Math.toRadians(152));
    private final Pose score789PoseBlue = new Pose(60.5,73.5, Math.toRadians(170));
    private final Pose grab101112PoseBlue  = grab789PoseBlue;
    private final Pose score101112PoseBlue = score789PoseBlue;
    private final Pose grab131415PoseBlue  = grab789PoseBlue;
    private final Pose score131415PoseBlue = score789PoseBlue;
    private final Pose prepGrab161718HeadingBlue = new Pose(0,0, Math.toRadians(262));
    private final Pose grab161718PoseBlue  = new Pose(17,35.4, Math.toRadians(180));
    private final Pose grab161718ControlPose1Blue = new Pose(55.4,35.5, 0);
    private final Pose grab161718ControlPose2Blue = new Pose(49.4,35, 0);
    private final Pose prepScore161718HeadingBlue = new Pose(0,0, Math.toRadians(238));
    private final Pose score161718PoseBlue = new Pose(47.7,84.7, Math.toRadians(180));
    private final Pose grab192021PoseBlue  = new Pose(18,84.5, Math.toRadians(180));
    private final Pose score192021PoseBlue = new Pose(48,84.5, Math.toRadians(180));
    private final Pose parkPoseBlue        = new Pose(46,83.5, Math.toRadians(225));

    /// RED SIDE POSES
    private final Pose startPoseRed     = startPoseBlue.mirror();
    private final Pose score123PoseRed  = score123PoseBlue.mirror();
    private final Pose prepGrab456PoseRed = prepGrab456PoseBlue.mirror();
    private final Pose grab456PoseRed   = grab456PoseBlue.mirror();
    private final Pose score456PoseRed  = score456PoseBlue.mirror();
    private final Pose grab789PoseRed   = grab789PoseBlue.mirror();
    private final Pose score789PoseRed  = score789PoseBlue.mirror();
    private final Pose grab101112PoseRed  = grab789PoseRed;
    private final Pose score101112PoseRed = score789PoseRed;
    private final Pose grab131415PoseRed  = grab789PoseRed;
    private final Pose score131415PoseRed = score789PoseRed;
    private final Pose prepGrab161718HeadingRed = prepGrab161718HeadingBlue.mirror();
    private final Pose grab161718PoseRed  = grab161718PoseBlue.mirror(); // TODO turnTo(278) before this path
    private final Pose grab161718ControlPose1Red = grab161718ControlPose1Blue.mirror();
    private final Pose grab161718ControlPose2Red = grab161718ControlPose2Blue.mirror();
    private final Pose prepScore161718HeadingRed = prepScore161718HeadingBlue.mirror();
    private final Pose score161718PoseRed = score161718PoseBlue.mirror(); // TODO turnTo(302) before this path
    private final Pose grab192021PoseRed  = grab192021PoseBlue.mirror();
    private final Pose score192021PoseRed = score192021PoseBlue.mirror();
    private final Pose parkPoseRed        = parkPoseBlue.mirror();

    // PathChains
    private PathChain BScore123, BGrab456, BScore456, BGrab789, BScore789;
    private PathChain BGrab101112, BScore101112, BGrab131415, BScore131415;
    private PathChain BGrab161718, BScore161718, BGrab192021, BScore192021, BPark;
    private PathChain RScore123, RGrab456, RScore456, RGrab789, RScore789;
    private PathChain RGrab101112, RScore101112, RGrab131415, RScore131415;
    private PathChain RGrab161718, RScore161718, RGrab192021, RScore192021, RPark;

    public void buildPaths() {
        /// BLUE SIDE
        BScore123 = follower.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, score123PoseBlue))
                .setLinearHeadingInterpolation(startPoseBlue.getHeading(), score123PoseBlue.getHeading())
//                .setGlobalDeceleration()
                .build();

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

        BGrab789 = follower.pathBuilder()
                .addPath(new BezierLine(score456PoseBlue, grab789PoseBlue))
                .setLinearHeadingInterpolation(score456PoseBlue.getHeading(), grab789PoseBlue.getHeading())
                .build();

        BScore789 = follower.pathBuilder()
                .addPath(new BezierLine(grab789PoseBlue, score789PoseBlue))
                .setLinearHeadingInterpolation(grab789PoseBlue.getHeading(), score789PoseBlue.getHeading())
                .build();

        BGrab101112 = follower.pathBuilder()
                .addPath(new BezierLine(score789PoseBlue, grab101112PoseBlue))
                .setLinearHeadingInterpolation(score789PoseBlue.getHeading(), grab101112PoseBlue.getHeading())
                .build();

        BScore101112 = follower.pathBuilder()
                .addPath(new BezierLine(grab101112PoseBlue, score101112PoseBlue))
                .setLinearHeadingInterpolation(grab101112PoseBlue.getHeading(), score101112PoseBlue.getHeading())
                .build();

        BGrab131415 = follower.pathBuilder()
                .addPath(new BezierLine(score101112PoseBlue, grab131415PoseBlue))
                .setLinearHeadingInterpolation(score101112PoseBlue.getHeading(), grab131415PoseBlue.getHeading())
                .build();

        BScore131415 = follower.pathBuilder()
                .addPath(new BezierLine(grab131415PoseBlue, score131415PoseBlue))
                .setLinearHeadingInterpolation(grab131415PoseBlue.getHeading(), score131415PoseBlue.getHeading())
                .build();

        BGrab161718 = follower.pathBuilder() /// Funny heading stuff before and after
                .addPath(new BezierCurve(score131415PoseBlue, grab161718ControlPose1Blue, grab161718ControlPose2Blue, grab161718PoseBlue))
                .setLinearHeadingInterpolation(prepGrab161718HeadingBlue.getHeading(), grab161718PoseBlue.getHeading(), 0.7)
                .build();

        BScore161718 = follower.pathBuilder() /// Funny heading stuff before
                .addPath(new BezierLine(grab161718PoseBlue, score161718PoseBlue))
                .setLinearHeadingInterpolation(prepScore161718HeadingBlue.getHeading(), score161718PoseBlue.getHeading())
                .build();

        BGrab192021 = follower.pathBuilder()
                .addPath(new BezierLine(score161718PoseBlue, grab192021PoseBlue))
                .setLinearHeadingInterpolation(score161718PoseBlue.getHeading(), grab192021PoseBlue.getHeading())
                .build();

        BScore192021 = follower.pathBuilder()
                .addPath(new BezierLine(grab192021PoseBlue, score192021PoseBlue))
                .setLinearHeadingInterpolation(grab192021PoseBlue.getHeading(), score192021PoseBlue.getHeading())
                .build();

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
//                .setGlobalDeceleration()
                .build();

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

        RGrab789 = follower.pathBuilder()
                .addPath(new BezierLine(score456PoseRed, grab789PoseRed))
                .setLinearHeadingInterpolation(score456PoseRed.getHeading(), grab789PoseRed.getHeading())
                .build();

        RScore789 = follower.pathBuilder()
                .addPath(new BezierLine(grab789PoseRed, score789PoseRed))
                .setLinearHeadingInterpolation(grab789PoseRed.getHeading(), score789PoseRed.getHeading())
                .build();

        RGrab101112 = follower.pathBuilder()
                .addPath(new BezierLine(score789PoseRed, grab101112PoseRed))
                .setLinearHeadingInterpolation(score789PoseRed.getHeading(), grab101112PoseRed.getHeading())
                .build();

        RScore101112 = follower.pathBuilder()
                .addPath(new BezierLine(grab101112PoseRed, score101112PoseRed))
                .setLinearHeadingInterpolation(grab101112PoseRed.getHeading(), score101112PoseRed.getHeading())
                .build();

        RGrab131415 = follower.pathBuilder()
                .addPath(new BezierLine(score101112PoseRed, grab131415PoseRed))
                .setLinearHeadingInterpolation(score101112PoseRed.getHeading(), grab131415PoseRed.getHeading())
                .build();

        RScore131415 = follower.pathBuilder()
                .addPath(new BezierLine(grab131415PoseRed, score131415PoseRed))
                .setLinearHeadingInterpolation(grab131415PoseRed.getHeading(), score131415PoseRed.getHeading())
                .build();

        RGrab161718 = follower.pathBuilder() /// Funny heading stuff before and after
                .addPath(new BezierCurve(score131415PoseRed, grab161718ControlPose1Red, grab161718ControlPose2Red, grab161718PoseRed))
                .setLinearHeadingInterpolation(score131415PoseRed.getHeading(), grab161718PoseRed.getHeading(), 0.7)
                .build();

        RScore161718 = follower.pathBuilder() /// Funny heading stuff before
                .addPath(new BezierLine(grab161718PoseRed, score161718PoseRed))
                .setLinearHeadingInterpolation(prepGrab161718HeadingRed.getHeading(), score161718PoseRed.getHeading())
                .build();

        RGrab192021 = follower.pathBuilder()
                .addPath(new BezierLine(score161718PoseRed, grab192021PoseRed))
                .setLinearHeadingInterpolation(score161718PoseRed.getHeading(), grab192021PoseRed.getHeading())
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
                cyclingFarZone = false; // redundant
                runShooter = true;
                runTurret = true;
                shooter.openLatch();
                follower.followPath(blueAlliance ? BScore123 : RScore123, true);
                setPathState(1);
                break;
            /// SCORE PRELOAD (123)
            case 1:
                if (shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && !currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
//                    runShooter = true;
                    shooter.closeLatch();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrab456 : RGrab456, 0.8, true);
                    setPathState(3);
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
                }
                break;
            /// SCORE S2 (456)
            case 4:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(6);
                }
                break;
            /// GRAB GATE (789)
            case 6:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    intake.deployIntake();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrab789 : RGrab789, true);
                    setPathState(7);
                }
                break;
            // OPEN GATE
            case 7:
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            // WAIT UNTIL INTAKE FULL OR TIME LIMIT PASSED
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > WAIT_GATE_DELAY || intake.isFull) {
                    intake.idle();
                    intake.stowIntake();
                    shooter.openLatch();
                    runShooter = true;
                    follower.followPath(blueAlliance ? BScore789 : RScore789, true);
                    setPathState(9);
                }
                break;
            /// SCORE GATE (789)
            case 9:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(10);
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
                }
                break;
            // OPEN GATE
            case 11:
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;
            // WAIT UNTIL INTAKE FULL OR TIME LIMIT PASSED
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > WAIT_GATE_DELAY || intake.isFull) {
                    intake.idle();
                    intake.stowIntake();
                    shooter.openLatch();
                    runShooter = true;
                    follower.followPath(blueAlliance ? BScore101112 : RScore101112, true);
                    setPathState(13);
                }
                break;
            /// SCORE GATE (101112)
            case 14:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(15);
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
                }
                break;
            // OPEN GATE
            case 16:
                if (!follower.isBusy()) {
                    setPathState(17);
                }
                break;
            // WAIT UNTIL INTAKE FULL OR TIME LIMIT PASSED
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > WAIT_GATE_DELAY || intake.isFull) {
                    intake.idle();
                    intake.stowIntake();
                    shooter.openLatch();
                    runShooter = true;
                    follower.followPath(blueAlliance ? BScore131415 : RScore131415, true);
                    setPathState(18);
                }
                break;
            /// SCORE GATE (131415)
            case 18:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    follower.turnTo(blueAlliance ? prepGrab161718HeadingBlue.getHeading() : prepGrab161718HeadingRed.getHeading()); /// Maybe broken or buggy
                    setPathState(19);
                }
                break;
            ///  GRAB S3 (161718)
            case 19:
                if (!follower.isTurning() /** might be deep fried */ && !currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    intake.deployIntake();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrab161718 : RGrab161718, true);
                    setPathState(20);
                }
                break;
            /// SCORE S3 (161718)
            case 20:
                if (!follower.isBusy() || intake.isFull) {
                    delayedIdleAction();
                    openLatchAction();
                    follower.turnTo(blueAlliance ? prepScore161718HeadingBlue.getHeading() : prepScore161718HeadingRed.getHeading()); /// Maybe broken or buggy
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isTurning() /** might be deep fried */) {
                    follower.followPath(blueAlliance ? BScore161718 : RScore161718, true);
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(23);
                }
                break;
            /// GRAB S1 (192021)
            case 23:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    intake.deployIntake();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrab192021 : RGrab192021, true);
                    setPathState(24);
                }
                break;
            /// SCORE S1 (192021)
            case 24:
                if (!follower.isBusy() || intake.isFull) {
                    delayedIdleAction();
                    openLatchAction();
                    follower.followPath(blueAlliance ? BScore192021 : RScore192021, true);
                    setPathState(25);
                }
                break;
            case 25:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(26);
                }
                break;
            /// PARK
            case 26:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    // park
                    follower.followPath(blueAlliance ? BPark : RPark, true);
                    setPathState(-1);
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
        shooter.initialize(this, robotHardware);
        turret.initialize(this, robotHardware);
        lights.initialize(this, robotHardware);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        // set servos for auto
        intake.deployIntake();
        shooter.closeLatch();
        lights.setColor(blueAlliance ? RGBLights.Colors.BLUE : RGBLights.Colors.RED);
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
        lights.operateAuto(); // rainbow strobe (can change endpoint colors and speed)
        autonomousPathUpdate();

        intake.operateAuto(currentlyShooting);
        shootWhileMoveCalcsSimple(currentPose);
//        shooter.operateAuto(currentPose.getX(), currentPose.getY(), blueAlliance, runShooter, cyclingFarZone);
//        turret.operateAuto(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()), blueAlliance, runTurret);


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
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");

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
                new InstantAction(() -> intake.runTransferOnly()),
                new SleepAction(TRANSFER_ONLY_DELAY),
                new InstantAction(() -> intake.shootingIntake(1)),
                new SleepAction(RAPID_FIRE_DELAY),
                new InstantAction(() -> intake.idle()),
                new InstantAction(() -> shooter.closeLatch()),
                new InstantAction(() -> currentlyShooting = false)
        ));
    }

    private void delayedIdleAction() {
        runningActions.add(new SequentialAction(
                new SleepAction(0.3),
                new InstantAction(() -> intake.idle())
        ));
    }

    private void openLatchAction() {
//        runningActions.add(new InstantAction(() -> shooter.openLatch()));
        runningActions.add(new SequentialAction(
                        new SleepAction(0.15),
                        new InstantAction(() -> shooter.openLatch())
                ));
    }

    private void shootWhileMoveCalcsSimple(Pose currentPose) { // short for calculator
        double temp_time = elapsedtime.milliseconds();

        double xVel = follower.getVelocity().getXComponent();
        double yVel = follower.getVelocity().getYComponent();
        double headingDegrees = Math.toDegrees(currentPose.getHeading());
        double currentXDist = (blueAlliance ? Globals.blueGoalX : Globals.redGoalX) - currentPose.getX();
        double currentYDist = (blueAlliance ? Globals.blueGoalY : Globals.redGoalY) - currentPose.getX();

        double currentDist = Math.hypot(currentXDist, currentYDist);

        telemetry.addLine("\n=== SHOOT WHILE MOVE CALCS ===");

        if (Math.abs(xVel) < 0.1 && Math.abs(xVel) < 0.1) {
            shooter.operateSWMAuto(currentXDist, currentYDist, blueAlliance, runShooter, cyclingFarZone);
            turret.operateSWMAuto(currentXDist, currentYDist, headingDegrees, runTurret);

            telemetry.addLine("0");
            telemetry.addLine("0");
            telemetry.addLine("0");
        }
        else {
            double airtime = -0.000012 * Math.pow(currentDist, 2) + 0.0036 * currentDist + 0.43;
            double adjustedXDist = currentXDist - (xVel * airtime);
            double adjustedYDist = currentYDist - (yVel * airtime);

            shooter.operateSWMSimple(adjustedXDist, adjustedYDist, runShooter, cyclingFarZone);
            turret.operateSWMSimple(adjustedXDist, adjustedYDist, headingDegrees, runTurret);

            telemetry.addData("airtime", airtime);
            telemetry.addData("adjustedXDist", adjustedXDist);
            telemetry.addData("adjustedYDist", adjustedYDist);
        }

        telemetry.addData("xVel", xVel);
        telemetry.addData("yVel", yVel);
        telemetry.addData("processing time taken", elapsedtime.milliseconds() - temp_time);
    }
}
