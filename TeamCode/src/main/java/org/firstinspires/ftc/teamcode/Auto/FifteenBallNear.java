//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.pedropathing.geometry.BezierCurve;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Subsystem.FlywheelShooter;
//import org.firstinspires.ftc.teamcode.Subsystem.Intake;
//import org.firstinspires.ftc.teamcode.Subsystem.LimelightVision;
//import org.firstinspires.ftc.teamcode.Subsystem.Turret;
//import org.firstinspires.ftc.teamcode.Util.Globals;
//import org.firstinspires.ftc.teamcode.Util.RobotHardware;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.Util.RGBLights;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@Autonomous(name = "15 Ball NEAR", group = "A", preselectTeleOp = "Odom TeleOp")
//public class FifteenBallNear extends OpMode {
//
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState;
//    private FtcDashboard dash = FtcDashboard.getInstance();
//    private List<Action> runningActions = new ArrayList<>();
//    private List<LynxModule> allHubs;
//    private ElapsedTime elapsedtime;
//
//
//    private RobotHardware robotHardware = new RobotHardware();
//    private Intake intake = new Intake();
//    private Turret turret = new Turret();
//    private FlywheelShooter shooter = new FlywheelShooter();
//    private LimelightVision llVision = new LimelightVision();
//    private RGBLights lights = new RGBLights();
//
//
//    ///  CONSTANTS
//    private boolean blueAlliance = true;
//    private int ballsLaunched = 0;
//    private volatile boolean runShooter = false;
//    private volatile boolean runTurret = false;
//    private volatile boolean cyclingFarZone = false;
//    private final double PUSHER_DELAY = 0.5; // seconds (sleepAction)
//    private final double RAPID_FIRE_DELAY = 0.6; // seconds (sleepAction)
//    private final double DELAY_BEFORE_MOVING = 150; // milliseconds
//    private final double HOLD_GATE_DELAY = 1000;
//    private final double SLOW_PATH_MAX_POWER = 0.6;
//
//    /// BLUE SIDE
//    private final Pose startPoseBlue = new Pose(30, 135.6, Math.toRadians(270));
//    private final Pose scorePoseBlue123 = new Pose(48, 96, Math.toRadians(235));
//    private final Pose pickupPoseBlue456 = new Pose(23, 83, Math.toRadians(190)); /// LOWER MAX POWER
//    private final Pose controlPoseBlue456 = new Pose(48, 86, 0);
//    private final Pose scorePoseBlue456 = new Pose(48, 96, Math.toRadians(235)); // same as 123
//    private final Pose prepPickupPoseBlue789 = new Pose(35.5, 69, Math.toRadians(235));
//    private final Pose pickupPoseBlue789 = new Pose(19, 61.5, Math.toRadians(210)); /// LOWER MAX POWER
////    private final Pose controlPoseBlue2 = new Pose(33.6, 60, 0);
//    private final Pose openGatePoseBlue = new Pose(13.5, 67, Math.toRadians(275)); /// LINEAR INTERPOLATION TO SCORE POSE EARLY
//    private final Pose scorePoseBlue789 = new Pose(53,87.6, Math.toRadians(210));
////    private final Pose openGateControlPoseBlue = new Pose(40, 77, Math.toRadians(180));
//    private final Pose prepPickupPoseBlue101112 = new Pose(37.5, 46, Math.toRadians(220)); /// LINEAR INTERPOLATION TURN EARLY
//    private final Pose pickupPoseBlue101112 = new Pose(24, 39, Math.toRadians(215)); /// LOWER MAX POWER
////    private final Pose controlPoseBlue3 = new Pose(31.5, 38.4, 0);
//    private final Pose scorePoseBlue101112 = new Pose(56.8, 82.2, Math.toRadians(228));
//    private final Pose prepPickupPoseBlue131415 = new Pose(12, 33, Math.toRadians(230));
//    private final Pose pickupPoseBlue131415 = new Pose(12, 10, Math.toRadians(230)); /// LOWER MAX POWER
//    private final Pose scorePoseBlue131415 = new Pose(59, 103, Math.toRadians(242));
////    private final Pose parkPoseBlue = new Pose(59, 103, Math.toRadians(235));
//
//    /// RED SIDE
//    private final Pose startPoseRed = startPoseBlue.mirror();
//    private final Pose scorePoseRed123 = scorePoseBlue123.mirror();
//    private final Pose pickupPoseRed456 = pickupPoseBlue456.mirror();
//    private final Pose controlPoseRed456 = controlPoseBlue456.mirror();
//    private final Pose scorePoseRed456 = scorePoseBlue456.mirror();
//    private final Pose prepPickupPoseRed789 = prepPickupPoseBlue789.mirror();
//    private final Pose pickupPoseRed789 = pickupPoseBlue789.mirror();
////    private final Pose controlPoseRed2 = controlPoseBlue2.mirror();
//    private final Pose openGatePoseRed = openGatePoseBlue.mirror();
//    private final Pose scorePoseRed789 = scorePoseBlue789.mirror();
//    private final Pose prepPickupPoseRed101112 = prepPickupPoseBlue101112.mirror();
//    private final Pose pickupPoseRed101112 = pickupPoseBlue101112.mirror();
////    private final Pose controlPoseRed3 = controlPoseBlue3.mirror();
//    private final Pose scorePoseRed101112 = scorePoseBlue101112.mirror();
//    private final Pose prepPickupPoseRed131415 = prepPickupPoseBlue131415.mirror();
//    private final Pose pickupPoseRed131415 = pickupPoseBlue131415.mirror();
//    private final Pose scorePoseRed131415 = scorePoseBlue131415.mirror();
////    private final Pose parkPoseRed = parkPoseBlue.mirror();
//
//    private PathChain startToScoreBlue123, pickupBallsBlue456, scorePickupBlue456, pickupBallsBlue789pt1, pickupBallsBlue789pt2, openGateBlue, scorePickupBlue789;
//    private PathChain pickupBallsBlue101112pt1, pickupBallsBlue101112pt2, scorePickupBlue101112, pickupBallsBlue131415pt1, pickupBallsBlue131415pt2, scorePickupBlue131415, parkBlue;
//
//    private PathChain startToScoreRed123, pickupBallsRed456, scorePickupRed456, pickupBallsRed789pt1, pickupBallsRed789pt2, openGateRed, scorePickupRed789;
//    private PathChain pickupBallsRed101112pt1, pickupBallsRed101112pt2, scorePickupRed101112, pickupBallsRed131415pt1, pickupBallsRed131415pt2, scorePickupRed131415, parkRed;
//
//    public void buildPaths() {
//        /// BLUE SIDE
//        startToScoreBlue123 = follower.pathBuilder()
//                .addPath(new BezierLine(startPoseBlue, scorePoseBlue123)).setLinearHeadingInterpolation(startPoseBlue.getHeading(), scorePoseBlue123.getHeading())
//                .build();
//
//        pickupBallsBlue456 = follower.pathBuilder() /// LOWER MAX POWER
//                .addPath(new BezierCurve(scorePoseBlue123, controlPoseBlue456, pickupPoseBlue456)).setLinearHeadingInterpolation(scorePoseBlue123.getHeading(), pickupPoseBlue456.getHeading())
//                .build();
//
//        scorePickupBlue456 = follower.pathBuilder()                                                                                         /// non-matching on purpose
//                .addPath(new BezierLine(pickupPoseBlue456, scorePoseBlue456)).setLinearHeadingInterpolation(pickupPoseBlue456.getHeading(), pickupPoseBlue456.getHeading())
//                .setGlobalDeceleration(1.1)
//                .build();
//
//        pickupBallsBlue789pt1 = follower.pathBuilder()                                                          /// non-matching on purpose
//                .addPath(new BezierLine(scorePoseBlue456, prepPickupPoseBlue789)).setLinearHeadingInterpolation(pickupPoseBlue456.getHeading(), prepPickupPoseBlue789.getHeading(), 0.3)
//                .build();
//
//        pickupBallsBlue789pt2 = follower.pathBuilder() /// LOWER MAX POWER
////                .addPath(new BezierCurve(prepPickupPoseBlue2, controlPoseBlue2, pickupPoseBlue2)).setLinearHeadingInterpolation(prepPickupPoseBlue2.getHeading(), pickupPoseBlue2.getHeading(), 0.3)
//                .addPath(new BezierLine(prepPickupPoseBlue789, pickupPoseBlue789)).setLinearHeadingInterpolation(prepPickupPoseBlue789.getHeading(), pickupPoseBlue789.getHeading())
//                .build();
//
//        openGateBlue = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPoseBlue789, openGatePoseBlue)).setLinearHeadingInterpolation(pickupPoseBlue789.getHeading(), openGatePoseBlue.getHeading())
//                .build();
//
//        scorePickupBlue789 = follower.pathBuilder() ///  EARLY TURN
//                .addPath(new BezierLine(openGatePoseBlue, scorePoseBlue789)).setLinearHeadingInterpolation(openGatePoseBlue.getHeading(), scorePoseBlue789.getHeading(), 0.3)
//                .setGlobalDeceleration(1.1)
//                .build();
//
//        pickupBallsBlue101112pt1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseBlue789, prepPickupPoseBlue101112)).setLinearHeadingInterpolation(scorePoseBlue789.getHeading(), prepPickupPoseBlue101112.getHeading(), 0.3)
//                .setGlobalDeceleration(1.1)
//                .build();
//
//        pickupBallsBlue101112pt2 = follower.pathBuilder() /// LOWER MAX POWER
////                .addPath(new BezierCurve(prepPickupPoseBlue101112, controlPoseBlue3, pickupPoseBlue101112)).setLinearHeadingInterpolation(prepPickupPoseBlue101112.getHeading(), pickupPoseBlue101112.getHeading(), 0.3)
//                .addPath(new BezierLine(prepPickupPoseBlue101112, pickupPoseBlue101112)).setLinearHeadingInterpolation(prepPickupPoseBlue101112.getHeading(), pickupPoseBlue101112.getHeading())
//                .build();
//
//        scorePickupBlue101112 = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPoseBlue101112, scorePoseBlue101112)).setLinearHeadingInterpolation(pickupPoseBlue101112.getHeading(), scorePoseBlue101112.getHeading(), 0.3)
//                .setGlobalDeceleration(1.1)
//                .build();
//
//        pickupBallsBlue131415pt1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseBlue101112, prepPickupPoseBlue131415)).setLinearHeadingInterpolation(scorePoseBlue101112.getHeading(), prepPickupPoseBlue131415.getHeading())
//                .setGlobalDeceleration(1.1)
//                .build();
//
//        pickupBallsBlue131415pt2 = follower.pathBuilder() /// LOWER MAX POWER
//                .addPath(new BezierLine(prepPickupPoseBlue131415, pickupPoseBlue131415)).setLinearHeadingInterpolation(prepPickupPoseBlue131415.getHeading(), pickupPoseBlue131415.getHeading(), 0.3)
//                .build();
//
//        scorePickupBlue131415 = follower.pathBuilder() ///  EARLY TURN
//                .addPath(new BezierLine(pickupPoseBlue131415, scorePoseBlue131415)).setLinearHeadingInterpolation(pickupPoseBlue131415.getHeading(), scorePoseBlue131415.getHeading(), 0.3)
//                .setGlobalDeceleration(1.1)
//                .build();
//
////        parkBlue = follower.pathBuilder()
////                .addPath(new BezierLine(scorePoseBlue, parkPoseBlue)).setLinearHeadingInterpolation(scorePoseBlue.getHeading(), parkPoseBlue.getHeading())
////                .build();
//
//        /// RED SIDE
//        startToScoreRed123 = follower.pathBuilder()
//                .addPath(new BezierLine(startPoseRed, scorePoseRed123)).setLinearHeadingInterpolation(startPoseRed.getHeading(), scorePoseRed123.getHeading())
//                .build();
//
//        pickupBallsRed456 = follower.pathBuilder() /// LOWER MAX POWER
//                .addPath(new BezierCurve(scorePoseRed123, controlPoseRed456, pickupPoseRed456)).setLinearHeadingInterpolation(scorePoseRed123.getHeading(), pickupPoseRed456.getHeading())
//                .build();
//
//        scorePickupRed456 = follower.pathBuilder()                                                                                    /// intentionally not matching
//                .addPath(new BezierLine(pickupPoseRed456, scorePoseRed456)).setLinearHeadingInterpolation(pickupPoseRed456.getHeading(), pickupPoseRed456.getHeading())
//                .setGlobalDeceleration(1.1)
//                .build();
//
//        pickupBallsRed789pt1 = follower.pathBuilder()                                                       /// intentionally not matching
//                .addPath(new BezierLine(scorePoseRed456, prepPickupPoseRed789)).setLinearHeadingInterpolation(pickupPoseRed456.getHeading(), prepPickupPoseRed789.getHeading(), 0.3)
//                .build();
//
//        pickupBallsRed789pt2 = follower.pathBuilder() /// LOWER MAX POWER
////                .addPath(new BezierCurve(prepPickupPoseRed789, controlPoseRed2, pickupPoseRed789)).setLinearHeadingInterpolation(prepPickupPoseRed789.getHeading(), pickupPoseRed789.getHeading(), 0.3)
//                .addPath(new BezierLine(prepPickupPoseRed789, pickupPoseRed789)).setLinearHeadingInterpolation(prepPickupPoseRed789.getHeading(), pickupPoseRed789.getHeading())
//                .build();
//
//        openGateRed = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPoseRed789, openGatePoseRed)).setLinearHeadingInterpolation(pickupPoseRed789.getHeading(), openGatePoseRed.getHeading())
//                .build();
//
//        scorePickupRed789 = follower.pathBuilder() ///  EARLY TURN
//                .addPath(new BezierLine(openGatePoseRed, scorePoseRed789)).setLinearHeadingInterpolation(openGatePoseRed.getHeading(), scorePoseRed789.getHeading(), 0.3)
//                .setGlobalDeceleration(1.1)
//                .build();
//
//        pickupBallsRed101112pt1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseRed789, prepPickupPoseRed101112)).setLinearHeadingInterpolation(scorePoseRed789.getHeading(), prepPickupPoseRed101112.getHeading(), 0.3)
//                .setGlobalDeceleration(1.1)
//                .build();
//
//        pickupBallsRed101112pt2 = follower.pathBuilder() /// LOWER MAX POWER
////                .addPath(new BezierCurve(prepPickupPoseRed101112, controlPoseRed3, pickupPoseRed101112)).setLinearHeadingInterpolation(prepPickupPoseRed101112.getHeading(), pickupPoseRed101112.getHeading(), 0.3)
//                .addPath(new BezierLine(prepPickupPoseRed101112, pickupPoseRed101112)).setLinearHeadingInterpolation(prepPickupPoseRed101112.getHeading(), pickupPoseRed101112.getHeading())
//                .build();
//
//        scorePickupRed101112 = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPoseRed101112, scorePoseRed101112)).setLinearHeadingInterpolation(pickupPoseRed101112.getHeading(), scorePoseRed101112.getHeading(), 0.3)
//                .setGlobalDeceleration(1.1)
//                .build();
//
//        pickupBallsRed131415pt1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseRed101112, prepPickupPoseRed131415)).setLinearHeadingInterpolation(scorePoseRed101112.getHeading(), prepPickupPoseRed131415.getHeading())
//                .setGlobalDeceleration(1.1)
//                .build();
//
//        pickupBallsRed131415pt2 = follower.pathBuilder() /// LOWER MAX POWER
//                .addPath(new BezierLine(prepPickupPoseRed131415, pickupPoseRed131415)).setLinearHeadingInterpolation(prepPickupPoseRed131415.getHeading(), pickupPoseRed131415.getHeading(), 0.3)
//                .build();
//
//        scorePickupRed131415 = follower.pathBuilder() ///  EARLY TURN
//                .addPath(new BezierLine(pickupPoseRed131415, scorePoseRed131415)).setLinearHeadingInterpolation(pickupPoseRed131415.getHeading(), scorePoseRed131415.getHeading(), 0.3)
//                .build();
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            /// BALLS 1, 2, 3
//            case 0: // start pose to score pose
//                follower.followPath(blueAlliance ? startToScoreBlue123 : startToScoreRed123, true);
//                setPathState(1);
//                break;
//            case 1: // spin up shooter separately to avoid brownout
//                if (!follower.isBusy()) {
//                    runTurret = true;
//                    runShooter = true;
//                    setPathState(2);
//                }
//                break;
//            case 2: // shooter up to speed, now shoot 3 balls
//                if (shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
//                    rapidFireAction();
//                    setPathState(3);
//                }
//                break;
//            case 3: // done rapid-firing, pick up next balls
//                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
//                    ballsLaunched = 3; // kinda deprecated now
//                    runShooter = false;
//                    runTurret = false;
//                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
//                    follower.followPath(blueAlliance ? pickupBallsBlue456 : pickupBallsRed456, SLOW_PATH_MAX_POWER, true);
//                    setPathState(4);
//                }
//                break;
//            ///  BALLS 4, 5, 6
//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(blueAlliance ? scorePickupBlue456 : scorePickupRed456,true);
//                    runningActions.add(new InstantAction(() -> intake.intake()));
//                    setPathState(5);
//                }
//                break;
//            case 5: // spin up shooter
//                if (!follower.isBusy()) {
//                    runTurret = true;
//                    runShooter = true;
//                    setPathState(6);
//                }
//                break;
//            case 6: // shooter up to speed, now shoot 3 balls
//                if (shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
//                    rapidFireAction();
//                    setPathState(7);
//                }
//                break;
//            case 7: // done rapid-firing, pick up next balls
//                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
//                    ballsLaunched = 6;
//                    runShooter = false;
//                    runTurret = false;
//                    runningActions.add(new InstantAction(() -> intake.intake()));
//                    follower.followPath(blueAlliance ? pickupBallsBlue789pt1 : pickupBallsRed789pt1, true);
//                    setPathState(8);
//                }
//                break;
//            ///  BALLS 7, 8, 9 + OPEN GATE
//            case 8:
//                if  (!follower.isBusy()) {
//                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
//                    follower.followPath(blueAlliance ? pickupBallsBlue789pt2 : pickupBallsRed789pt2, SLOW_PATH_MAX_POWER, true);
//                    setPathState(9);
//                }
//                break;
//            case 9: // balls grabbed, now open gate
//                if (!follower.isBusy() || intake.motorStalled) {
//                    delayedIdle();
//                    follower.followPath(blueAlliance ? openGateBlue : openGateRed, true);
//                    setPathState(10);
//                }
//                break;
//            case 10: // opened gate
//                if (!follower.isBusy()) {
//                    setPathState(11);
//                }
//                break;
//            case 11: // short delay to let balls out, then return to score
//                if (pathTimer.getElapsedTime() > HOLD_GATE_DELAY) {
//                    runShooter = true;
//                    follower.followPath(blueAlliance ? scorePickupBlue789 : scorePickupRed789,true);
//                    setPathState(12);
//                }
//                break;
//            case 12: // spin up shooter
//                if (!follower.isBusy()) {
//                    runShooter = true;
//                    runTurret = true;
//                    setPathState(13);
//                }
//                break;
//            case 13: // shooter up to speed, now shoot next 3 balls
//                if (shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
//                    rapidFireAction();
//                    setPathState(14);
//                }
//                break;
//            case 14: // done rapid-firing, pick up next balls
//                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
//                    ballsLaunched = 9;
//                    runShooter = false;
//                    runTurret = false;
//                    follower.followPath(blueAlliance ? pickupBallsBlue101112pt1 : pickupBallsRed101112pt1,true);
//                    setPathState(15);
//                }
//                break;
//            ///  BALLS 10, 11, 12
//            case 15: // pick up slowly
//                if (!follower.isBusy()) {
//                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
//                    follower.followPath(blueAlliance ? pickupBallsBlue101112pt2 : pickupBallsRed101112pt2, SLOW_PATH_MAX_POWER, true);
//                    setPathState(16);
//                }
//                break;
//            case 16: // balls grabbed, now return to score
//                if (!follower.isBusy()) {
//                    runShooter = true;
//                    delayedIdle();
//                    follower.followPath(blueAlliance ? scorePickupBlue101112 : scorePickupRed101112, true);
//                    setPathState(17);
//                }
//                break;
//            case 17: // spin up shooter
//                if (!follower.isBusy()) {
//                    runShooter = true;
//                    runTurret = true;
//                    setPathState(18);
//                }
//                break;
//            case 18: // shooter up to speed, now shoot next 3 balls
//                if (shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
//                    rapidFireAction();
//                    setPathState(19);
//                }
//                break;
//            case 19: // done rapid-firing, pick up next balls
//                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
//                    ballsLaunched = 12;
//                    runShooter = false;
//                    runTurret = false;
//                    runningActions.add(new InstantAction(() -> intake.intake()));
//                    follower.followPath(blueAlliance ? pickupBallsBlue131415pt1 : pickupBallsRed131415pt1,true);
//                    setPathState(20);
//                }
//                break;
//            /// BALLS 13, 14, 15
//            case 20: // pick up slowly
////                if (intake.motorStalled) { // already have 3 in chamber, skip a path and go back to score
////                    runningActions.add(new InstantAction(() -> intake.idle()));
////                    follower.followPath(blueAlliance ? scorePickupBlue131415 : scorePickupRed131415, true);
////                    setPathState(22);
////                }
////                else
//                if (!follower.isBusy() || follower.isRobotStuck()) {
//                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
//                    follower.followPath(blueAlliance ? pickupBallsBlue131415pt2 : pickupBallsRed131415pt2, SLOW_PATH_MAX_POWER, true);
//                    setPathState(21);
//                }
//                break;
//            case 21: // balls grabbed, now return to score
//                if (!follower.isBusy() || follower.isRobotStuck() || pathTimer.getElapsedTime() > 2300) {
//                    delayedIdle();
//                    follower.followPath(blueAlliance ? scorePickupBlue131415 : scorePickupRed131415, true);
//                    setPathState(22);
//                }
//                break;
//            case 22: // spin up shooter
//                if (!follower.isBusy()) {
//                    runningActions.add(new InstantAction(() -> intake.idle()));
//                    runShooter = true;
//                    runTurret = true;
//                    setPathState(23);
//                }
//                break;
//            case 23: // shooter up to speed, now shoot next 3 balls
//                if (shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
//                    rapidFireAction();
//                    setPathState(24);
//                }
//                break;
//            /// PARK
//            case 24: // done rapid-firing, already parked
//                if (!shooter.shooterLatchOpen) {
//                    runShooter = false; // back to idle, will turn off when opmode stops
//                    runTurret = false; // return to center
//                    ballsLaunched = 15; // deprecated
//                    runningActions.add(new SequentialAction( // technically redundant, but doesn't hurt to have
//                            new InstantAction(() -> intake.idle()),
//                            new InstantAction(() -> intake.stowBallPusher()),
//                            new InstantAction(() -> shooter.closeLatch())
//                    ));
//                    setPathState(-1); // all done
//                }
//                break;
//        }
//    }
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//        robotHardware.initialize(this);
//        intake.initialize(this, robotHardware);
//        llVision.initialize(this, robotHardware);
//        shooter.initialize(this, robotHardware, llVision);
//        turret.initialize(this, robotHardware, llVision, true);
//        lights.initialize(this, robotHardware);
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//
//        elapsedtime = new ElapsedTime();
//        elapsedtime.reset();
//
//        allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }
//
//        // set servos for auto
//        intake.stowBallPusher();
//        shooter.closeLatch();
//        lights.setColor(blueAlliance ? RGBLights.Colors.BLUE : RGBLights.Colors.RED);
//    }
//
//    @Override
//    public void init_loop() {
//        if (gamepad1.b || gamepad2.b) {
//            blueAlliance = false;
//            lights.setColor(RGBLights.Colors.RED);
//        } else if (gamepad1.x || gamepad2.x) {
//            blueAlliance = true;
//            lights.setColor(RGBLights.Colors.BLUE);
//        }
//
//        follower.update();
//
//        telemetry.addLine("B for RED | X for BLUE");
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
//        telemetry.addData("x ", follower.getPose().getX());
//        telemetry.addData("y ", follower.getPose().getY());
//        telemetry.addData("heading ", follower.getPose().getHeading());
//        if (Math.abs(follower.getVelocity().getMagnitude()) > 1 || Math.abs(follower.getAngularVelocity()) > 0.2) {
//            telemetry.addLine("PINPOINT IS COOKED");
//            telemetry.addLine("PINPOINT IS COOKED");
//            telemetry.addLine("PINPOINT IS COOKED");
//            telemetry.addLine("PINPOINT IS COOKED");
//            telemetry.addLine("PINPOINT IS COOKED");
//        }
//        telemetry.addData("velocity magnitude", Math.abs(follower.getVelocity().getMagnitude()));
//        telemetry.addData("angular velocity", Math.abs(follower.getAngularVelocity()));
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        follower.setStartingPose(blueAlliance ? startPoseBlue : startPoseRed);
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    @Override
//    public void loop() {
//        for (LynxModule hub : allHubs) { hub.clearBulkCache(); }
//
//        // RR Actions
//        TelemetryPacket packet = new TelemetryPacket();
//        List<Action> newActions = new ArrayList<>();
//        for (Action action : runningActions) { if (action.run(packet)) { newActions.add(action); } }
//        runningActions = newActions;
//
//        // loops
//        intake.operateCurrentLimiting();
//        shooter.operateOdomAuto(follower.getPose().getX(), follower.getPose().getY(), blueAlliance, runShooter, cyclingFarZone);
//        turret.operateOdomAuto(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()), blueAlliance, runTurret);
//        follower.update();
//        lights.operateAuto(); // rainbow strobe (can change endpoint colors and speed)
//        autonomousPathUpdate();
//
//        // Feedback to Driver Hub for debugging
//        telemetry.addLine("STATE TELEMETRY");
//        telemetry.addData("path state ", pathState);
//        telemetry.addData("runShooter? ", runShooter);
//        telemetry.addData("runTurret? ", runTurret);
//        telemetry.addLine("\n");
//        telemetry.addLine("PEDRO TELEMETRY");
//        telemetry.addData("isBusy? ", follower.isBusy());
//        telemetry.addData("isRobotStuck? ", follower.isRobotStuck());
//        telemetry.addData("x ", follower.getPose().getX());
//        telemetry.addData("y ", follower.getPose().getY());
//        telemetry.addData("heading ", Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.addLine("\n");
//        telemetry.addData("looptimes ", elapsedtime.milliseconds());
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
//
//        elapsedtime.reset();
//    }
//
//    @Override
//    public void stop() {
//        Globals.autoEndPose = follower.getPose();
//        Globals.blueAlliance = blueAlliance;
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    // method for rapid firing to reduce
//    private void rapidFireAction() {
//        runningActions.add(new SequentialAction(
//                new InstantAction(() -> shooter.openLatch()),
//                new InstantAction(() -> intake.intakeFullPower()),
//                new SleepAction(RAPID_FIRE_DELAY),
//                new InstantAction(() -> intake.deployBallPusher()),
//                new SleepAction(PUSHER_DELAY),
//                new InstantAction(() -> pathTimer.resetTimer()),
//                new InstantAction(() -> intake.stowBallPusher()),
//                new InstantAction(() -> shooter.closeLatch()),
//                new InstantAction(() -> intake.idle())
//        ));
//    }
//    private void delayedIdle() {
//        runningActions.add(new SequentialAction(
//                new InstantAction(() -> intake.intake()),
//                new SleepAction(1.5),
//                new InstantAction(() -> intake.idle())
//        ));
//    }
//}
