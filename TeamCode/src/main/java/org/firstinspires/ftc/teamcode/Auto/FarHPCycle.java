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

@Autonomous(name = "Far HP Cycle auto", group = "A", preselectTeleOp = "Full Teleop DUAL DRIVER")
public class FarHPCycle extends OpMode {

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
    private volatile boolean currentlyShooting = false;
    private boolean cyclingFarZone = true; // for shooter
    private boolean grabS3 = false;

    ///  CONSTANTS
    private final double TRANSFER_ONLY_DELAY = 0.03;
    private final double RAPID_FIRE_DELAY = 0.6; // seconds (sleepAction)
    private final double DELAY_BEFORE_MOVING = 150; // milliseconds

    /// BLUE SIDE
    private final Pose startPoseBlue = new Pose(54.3,8.3, Math.toRadians(180));
    private final Pose scorePoseBlue = new Pose(50.5, 11.5, Math.toRadians(180));
    private final Pose prepGrabS3PoseBlue = new Pose(45, 35.5, Math.toRadians(180));
    private final Pose grabS3PoseBlue = new Pose(15, 35.5, Math.toRadians(180));
    private final Pose hpPrepPoseBlueCorner = new Pose(18.5, 8, Math.toRadians(180));
    private final Pose hpGrabPoseBlueCorner = new Pose(12.5, 8, Math.toRadians(180));
    private final Pose hpPrepPoseBlueMiddle = new Pose(18.5, 16, Math.toRadians(180));
    private final Pose hpGrabPoseBlueMiddle = new Pose(12.5, 16, Math.toRadians(180));
    private final Pose hpPrepPoseBlueHigh   = new Pose(12.5, 24, Math.toRadians(230));
    private final Pose hpGrabControlPoseBlueHigh = new Pose(9.5, 18.5, 0);
    private final Pose hpGrabPoseBlueHigh   = new Pose(9.5, 12.5, Math.toRadians(270));
    private final Pose parkPoseBlue = new Pose(49, 13, Math.toRadians(135));

    /// RED SIDE
    private final Pose startPoseRed = startPoseBlue.mirror();
    private final Pose scorePoseRed = scorePoseBlue.mirror();
    private final Pose prepGrabS3PoseRed = prepGrabS3PoseBlue.mirror();
    private final Pose grabS3PoseRed = grabS3PoseBlue.mirror();
    private final Pose hpPrepPoseRedCorner = hpPrepPoseBlueCorner.mirror();
    private final Pose hpGrabPoseRedCorner = hpGrabPoseBlueCorner.mirror();
    private final Pose hpPrepPoseRedMiddle = hpPrepPoseBlueMiddle.mirror();
    private final Pose hpGrabPoseRedMiddle = hpGrabPoseBlueMiddle.mirror();
    private final Pose hpPrepPoseRedHigh   = hpPrepPoseBlueHigh.mirror();
    private final Pose hpGrabControlPoseRedHigh = hpGrabControlPoseBlueHigh.mirror();
    private final Pose hpGrabPoseRedHigh   = hpGrabPoseBlueHigh.mirror();
    private final Pose parkPoseRed = parkPoseBlue.mirror();

    // path chains:
    private PathChain BScore123, BGrabHPCorner, BScoreHPCorner, BGrabHPMiddle, BScoreHPMiddle, BGrabHPHigh, BScoreHPHigh, BGrabS3, BScoreS3, BPark;
    /// add the option to add waits and whether S3
    private PathChain RScore123, RGrabHPCorner, RScoreHPCorner, RGrabHPMiddle, RScoreHPMiddle, RGrabHPHigh, RScoreHPHigh, RGrabS3, RScoreS3, RPark;

    public void buildPaths() {
        /// BLUE SIDE
        BScore123 = follower.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, scorePoseBlue))
                .setLinearHeadingInterpolation(startPoseBlue.getHeading(), scorePoseBlue.getHeading())
                .build();

        BGrabS3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseBlue, prepGrabS3PoseBlue))
                .setLinearHeadingInterpolation(scorePoseBlue.getHeading(), prepGrabS3PoseBlue.getHeading())
                .addPath(new BezierLine(prepGrabS3PoseBlue, grabS3PoseBlue))
                .setLinearHeadingInterpolation(prepGrabS3PoseBlue.getHeading(), grabS3PoseBlue.getHeading())
                .build();

        BScoreS3 = follower.pathBuilder()
                .addPath(new BezierLine(grabS3PoseBlue, scorePoseBlue))
                .setLinearHeadingInterpolation(grabS3PoseBlue.getHeading(), scorePoseBlue.getHeading())
                .build();

        BGrabHPCorner = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseBlue, hpPrepPoseBlueCorner))
                .setLinearHeadingInterpolation(scorePoseBlue.getHeading(), hpPrepPoseBlueCorner.getHeading())
                .addPath(new BezierLine(hpPrepPoseBlueCorner, hpGrabPoseBlueCorner))
                .setLinearHeadingInterpolation(hpPrepPoseBlueCorner.getHeading(), hpGrabPoseBlueCorner.getHeading())
                .build();

        BScoreHPCorner = follower.pathBuilder()
                .addPath(new BezierLine(hpGrabPoseBlueCorner, scorePoseBlue))
                .setLinearHeadingInterpolation(hpGrabPoseBlueCorner.getHeading(), scorePoseBlue.getHeading())
                .build();

        BGrabHPMiddle = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseBlue, hpPrepPoseBlueMiddle))
                .setLinearHeadingInterpolation(scorePoseBlue.getHeading(), hpPrepPoseBlueMiddle.getHeading())
                .addPath(new BezierLine(hpPrepPoseBlueMiddle, hpGrabPoseBlueMiddle))
                .setLinearHeadingInterpolation(hpPrepPoseBlueMiddle.getHeading(), hpGrabPoseBlueMiddle.getHeading())
                .build();

        BScoreHPMiddle = follower.pathBuilder()
                .addPath(new BezierLine(hpGrabPoseBlueMiddle, scorePoseBlue))
                .setLinearHeadingInterpolation(hpGrabPoseBlueMiddle.getHeading(), scorePoseBlue.getHeading())
                .build();

        BGrabHPHigh = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseBlue, hpPrepPoseBlueHigh))
                .setLinearHeadingInterpolation(scorePoseBlue.getHeading(), hpPrepPoseBlueHigh.getHeading())
                .addPath(new BezierCurve(hpPrepPoseBlueHigh, hpGrabControlPoseBlueHigh, hpGrabPoseBlueHigh))
                .setLinearHeadingInterpolation(hpPrepPoseBlueHigh.getHeading(), hpGrabPoseBlueHigh.getHeading())
                .build();

        BScoreHPHigh = follower.pathBuilder()
                .addPath(new BezierLine(hpGrabPoseBlueHigh, scorePoseBlue))
                .setLinearHeadingInterpolation(hpGrabPoseBlueHigh.getHeading(), scorePoseBlue.getHeading())
                .build();

        BPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseBlue, parkPoseBlue))
                .setLinearHeadingInterpolation(scorePoseBlue.getHeading(), parkPoseBlue.getHeading())
                .build();

        /// RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE
        /// RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE
        /// RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE RED SIDE

        RScore123 = follower.pathBuilder()
                .addPath(new BezierLine(startPoseRed, scorePoseRed))
                .setLinearHeadingInterpolation(startPoseRed.getHeading(), scorePoseRed.getHeading())
                .build();

        RGrabS3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseRed, prepGrabS3PoseRed))
                .setLinearHeadingInterpolation(scorePoseRed.getHeading(), prepGrabS3PoseRed.getHeading())
                .addPath(new BezierLine(prepGrabS3PoseRed, grabS3PoseRed))
                .setLinearHeadingInterpolation(prepGrabS3PoseRed.getHeading(), grabS3PoseRed.getHeading())
                .build();

        RScoreS3 = follower.pathBuilder()
                .addPath(new BezierLine(grabS3PoseRed, scorePoseRed))
                .setLinearHeadingInterpolation(grabS3PoseRed.getHeading(), scorePoseRed.getHeading())
                .build();

        RGrabHPCorner = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseRed, hpPrepPoseRedCorner))
                .setLinearHeadingInterpolation(scorePoseRed.getHeading(), hpPrepPoseRedCorner.getHeading())
                .addPath(new BezierLine(hpPrepPoseRedCorner, hpGrabPoseRedCorner))
                .setLinearHeadingInterpolation(hpPrepPoseRedCorner.getHeading(), hpGrabPoseRedCorner.getHeading())
                .build();

        RScoreHPCorner = follower.pathBuilder()
                .addPath(new BezierLine(hpGrabPoseRedCorner, scorePoseRed))
                .setLinearHeadingInterpolation(hpGrabPoseRedCorner.getHeading(), scorePoseRed.getHeading())
                .build();

        RGrabHPMiddle = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseBlue, hpPrepPoseRedMiddle))
                .setLinearHeadingInterpolation(scorePoseBlue.getHeading(), hpPrepPoseRedMiddle.getHeading())
                .addPath(new BezierLine(hpPrepPoseRedMiddle, hpGrabPoseRedMiddle))
                .setLinearHeadingInterpolation(hpPrepPoseRedMiddle.getHeading(), hpGrabPoseRedMiddle.getHeading())
                .build();

        RScoreHPMiddle = follower.pathBuilder()
                .addPath(new BezierLine(hpGrabPoseRedMiddle, scorePoseRed))
                .setLinearHeadingInterpolation(hpGrabPoseRedMiddle.getHeading(), scorePoseRed.getHeading())
                .build();


        RGrabHPHigh = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseBlue, hpPrepPoseRedHigh))
                .setLinearHeadingInterpolation(scorePoseBlue.getHeading(), hpPrepPoseRedHigh.getHeading())
                .addPath(new BezierCurve(hpPrepPoseRedHigh, hpGrabControlPoseRedHigh, hpGrabPoseRedHigh))
                .setLinearHeadingInterpolation(hpPrepPoseRedHigh.getHeading(), hpGrabPoseRedHigh.getHeading())
                .build();

        RScoreHPHigh = follower.pathBuilder()
                .addPath(new BezierLine(hpGrabPoseRedHigh, scorePoseRed))
                .setLinearHeadingInterpolation(hpGrabPoseRedHigh.getHeading(), scorePoseRed.getHeading())
                .build();

        RPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseRed, parkPoseRed))
                .setLinearHeadingInterpolation(scorePoseRed.getHeading(), parkPoseRed.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            ///  SPIN UP + SCOOCH
            case 0:
                cyclingFarZone = true;
                runShooter = true;
                runTurret = true;
                shooter.openLatch();
                follower.followPath(blueAlliance ? BScore123 : RScore123, 0.5, true);
                setPathState(1);
                break;
            /// SCORE PRELOAD (123)
            case 1:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    if (grabS3) {
                        setPathState(100);
                    } else {
                        setPathState(2);
                    }
                }
                break;
            /// IF WE WANT, GRAB S3
            case 100:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    shooter.closeLatch();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrabS3 : RGrabS3, false);
                    setPathState(101);
                }
                break;
            case 101:
                if (!follower.isBusy() || intake.isFull) {
                    delayedIdleAction();
                    openLatchAction();
                    runShooter = true;
                    follower.followPath(blueAlliance ? BScoreS3 : RScoreS3, true);
                    setPathState(102);
                }
                break;
            /// SCORE S3
            case 102:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(2);
                }
                break;
            /// GRAB CORNER
            case 2:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    shooter.closeLatch();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrabHPCorner : RGrabHPCorner, false);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() || intake.isFull) {
                    delayedIdleAction();
                    openLatchAction();
                    runShooter = true;
                    follower.followPath(blueAlliance ? BScoreHPCorner : RScoreHPCorner, true);
                    setPathState(4);
                }
                break;
            /// SCORE CORNER
            case 4:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(5);
                }
                break;
            /// GRAB HIGH
            case 5:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    shooter.closeLatch();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrabHPHigh : RGrabHPHigh, false); // TODO
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() || intake.isFull) {
                    delayedIdleAction();
                    openLatchAction();
                    runShooter = true;
                    follower.followPath(blueAlliance ? BScoreHPHigh : RScoreHPHigh, true); // TODO
                    setPathState(7);
                }
                break;
            /// SCORE HIGH
            case 7:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(8);
                }
                break;
            /// GRAB MIDDLE
            case 8:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    shooter.closeLatch();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrabHPMiddle : RGrabHPMiddle, false); // TODO
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() || intake.isFull) {
                    delayedIdleAction();
                    openLatchAction();
                    runShooter = true;
                    follower.followPath(blueAlliance ? BScoreHPMiddle : RScoreHPMiddle, true); // TODO
                    setPathState(10);
                }
                break;
            /// SCORE MIDDLE
            case 10:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(11);
                }
                break;
            /// GRAB CORNER
            case 11:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    shooter.closeLatch();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrabHPCorner : RGrabHPCorner, false); // TODO
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() || intake.isFull) {
                    delayedIdleAction();
                    openLatchAction();
                    runShooter = true;
                    follower.followPath(blueAlliance ? BScoreHPCorner : RScoreHPCorner, true); // TODO
                    setPathState(13);
                }
                break;
            /// SCORE CORNER
            case 13:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    if (grabS3) {
                        /// park
                        setPathState(14);
                    } else {
                        /// run one more cycle
                        setPathState(11);
                        grabS3 = true; // kinda goofy, but avoids infinite loop without adding extra variable
                    }
                }
                break;
            /// PARK
            case 14:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
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
                    setPathState(-2);
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

        // toggles whether to grab spike 3
        if (gamepad1.aWasPressed()) {
            grabS3 = !grabS3;
        }
        follower.update();

        telemetry.addLine("B for RED | X for BLUE");
        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
        telemetry.addData("\ngrabS3?", grabS3);

        telemetry.addData("\nx ", follower.getPose().getX());
        telemetry.addData("y ", follower.getPose().getY());
        telemetry.addData("heading ", Math.toDegrees(follower.getPose().getHeading()));
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
        shooter.operateAuto(currentPose.getX(), currentPose.getY(), blueAlliance, runShooter, cyclingFarZone);
        turret.operateAuto(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()), blueAlliance, runTurret);

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
                new InstantAction(() -> intake.shootingIntake()),
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
        runningActions.add(new InstantAction(() -> shooter.openLatch()));
    }
}
