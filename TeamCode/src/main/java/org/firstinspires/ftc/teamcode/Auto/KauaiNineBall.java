//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
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
//
//import java.util.ArrayList;
//import java.util.List;
//
//@Autonomous(name = "Kauai 9 Ball", group = "A")
//public class KauaiNineBall extends OpMode {
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
//
//
//    ///  CONSTANTS
//    private boolean blueAlliance = true;
//    private int ballsLaunched = 0;
//    private volatile boolean runShooter = false;
//    private volatile boolean runTurret = false;
//    //    private final double MINIMUM_RPM = 2300;
//    private final double PUSHER_DELAY = 0.5;
//    private final double TIME_BETWEEN_SHOTS = 700; // TODO: shorten as much as possible
//
//    /// BLUE SIDE
//    private final Pose startPoseBlue = new Pose(25.5, 128, Math.toRadians(145));
//    private final Pose scorePoseBlue = new Pose(61, 84, Math.toRadians(180));
//    private final Pose pickupPoseBlue1 = new Pose(20, 84, Math.toRadians(180));
////    private final Pose controlPickupPoseBlue2 = new Pose(60.3, 55.8, Math.toRadians(180));
////    private final Pose control2PickupPoseBlue2 = new Pose(66.8, 57.7, Math.toRadians(180));
//    private final Pose prepPickupPoseBlue2 = new Pose(51, 63, Math.toRadians(180));
//    private final Pose pickupPoseBlue2 = new Pose(20, 60, Math.toRadians(180));
//    private final Pose prepPickupPoseBlue3 = new Pose(51, 40, Math.toRadians(180));
//    private final Pose pickupPoseBlue3 = new Pose(20, 36, Math.toRadians(180));
//    private final Pose parkPoseBlue = new Pose(28, 72, Math.toRadians(180));
//
//    /// RED SIDE
//    private final Pose startPoseRed = startPoseBlue.mirror();
//    private final Pose scorePoseRed = new Pose(83, 84, Math.toRadians(0));
//    private final Pose pickupPoseRed1 = pickupPoseBlue1.mirror();
////    private final Pose controlPickupPoseRed2 = controlPickupPoseBlue2.mirror();
////    private final Pose control2PickupPoseRed2 = control2PickupPoseBlue2.mirror();
//    private final Pose prepPickupPoseRed2 = prepPickupPoseBlue2.mirror();
//    private final Pose pickupPoseRed2 = pickupPoseBlue2.mirror();
//    private final Pose prepPickupPoseRed3 = prepPickupPoseBlue3.mirror();
//    private final Pose pickupPoseRed3 = pickupPoseBlue3.mirror();
//    private final Pose parkPoseRed = parkPoseBlue.mirror();
//
//    private PathChain startToScoreBlue, pickupBallsBlue1, scorePickupBlue1, pickupBallsBlue2, scorePickupBlue2, pickupBallsBlue3, parkBlue;
//    private PathChain startToScoreRed, pickupBallsRed1, scorePickupRed1, pickupBallsRed2, scorePickupRed2, pickupBallsRed3, parkRed;
//
//    public void buildPaths() {
//        /// BLUE SIDE
//        startToScoreBlue = follower.pathBuilder()
//                .addPath(new BezierLine(startPoseBlue, scorePoseBlue)).setLinearHeadingInterpolation(startPoseBlue.getHeading(), scorePoseBlue.getHeading())
//                .build();
//
//        pickupBallsBlue1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseBlue, pickupPoseBlue1)).setLinearHeadingInterpolation(scorePoseBlue.getHeading(), pickupPoseBlue1.getHeading())
//                .setBrakingStart(12)
//                .build();
//
//        scorePickupBlue1 = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPoseBlue1, scorePoseBlue)).setLinearHeadingInterpolation(pickupPoseBlue1.getHeading(), scorePoseBlue.getHeading())
//                .build();
//
//        // curve
////        pickupBallsBlue2 = follower.pathBuilder()
////                .addPath(new BezierCurve(scorePoseBlue, controlPickupPoseBlue2, control2PickupPoseBlue2, pickupPoseBlue2))
////                .setLinearHeadingInterpolation(scorePoseBlue.getHeading(), pickupPoseBlue2.getHeading())
////                .setBrakingStart(14)
////                .build();
//        pickupBallsBlue2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseBlue, prepPickupPoseBlue2)).setLinearHeadingInterpolation(scorePoseBlue.getHeading(), prepPickupPoseBlue2.getHeading())
//                .setBrakingStart(10)
//                .addPath(new BezierLine(prepPickupPoseBlue2, pickupPoseBlue2)).setLinearHeadingInterpolation(prepPickupPoseBlue2.getHeading(), pickupPoseBlue2.getHeading())
//                .setBrakingStart(10)
//                .build();
//
//
//        scorePickupBlue2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPoseBlue2, scorePoseBlue)).setLinearHeadingInterpolation(pickupPoseBlue2.getHeading(), scorePoseBlue.getHeading())
//                .build();
//
//        pickupBallsBlue3 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseBlue, prepPickupPoseBlue3)).setLinearHeadingInterpolation(scorePoseBlue.getHeading(), prepPickupPoseBlue3.getHeading())
//                .setBrakingStart(10)
//                .addPath(new BezierLine(prepPickupPoseBlue3, pickupPoseBlue3)).setLinearHeadingInterpolation(prepPickupPoseBlue3.getHeading(), pickupPoseBlue3.getHeading())
//                .setBrakingStart(10)
//                .build();
//
//        parkBlue = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPoseBlue3, parkPoseBlue)).setLinearHeadingInterpolation(pickupPoseBlue3.getHeading(), parkPoseBlue.getHeading())
//                .build();
//
//        /// RED SIDE
//        startToScoreRed = follower.pathBuilder()
//                .addPath(new BezierLine(startPoseRed, scorePoseRed)).setLinearHeadingInterpolation(startPoseRed.getHeading(), scorePoseRed.getHeading())
//                .build();
//
//        pickupBallsRed1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseRed, pickupPoseRed1)).setLinearHeadingInterpolation(scorePoseRed.getHeading(), pickupPoseRed1.getHeading())
//                .setBrakingStart(12)
//                .build();
//
//        scorePickupRed1 = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPoseRed1, scorePoseRed)).setLinearHeadingInterpolation(pickupPoseRed1.getHeading(), scorePoseRed.getHeading())
//                .build();
//
//        // curve
////        pickupBallsRed2 = follower.pathBuilder()
////                .addPath(new BezierCurve(scorePoseRed, controlPickupPoseRed2, control2PickupPoseRed2, pickupPoseRed2))
////                .setLinearHeadingInterpolation(scorePoseBlue.getHeading(), pickupPoseBlue2.getHeading())
////                .setBrakingStart(14)
////                .build();
//        pickupBallsRed2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseRed, prepPickupPoseRed2)).setLinearHeadingInterpolation(scorePoseRed.getHeading(), prepPickupPoseRed2.getHeading())
//                .setBrakingStart(10)
//                .addPath(new BezierLine(prepPickupPoseRed2, pickupPoseRed2)).setLinearHeadingInterpolation(prepPickupPoseRed2.getHeading(), pickupPoseRed2.getHeading())
//                .setBrakingStart(10)
//                .build();
//
//        scorePickupRed2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPoseRed2, scorePoseRed)).setLinearHeadingInterpolation(pickupPoseRed2.getHeading(), scorePoseRed.getHeading())
//                .build();
//
//        pickupBallsRed3 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseRed, prepPickupPoseRed3)).setLinearHeadingInterpolation(scorePoseRed.getHeading(), prepPickupPoseRed3.getHeading())
//                .setBrakingStart(10)
//                .addPath(new BezierLine(prepPickupPoseRed3, pickupPoseRed3)).setLinearHeadingInterpolation(prepPickupPoseRed3.getHeading(), pickupPoseRed3.getHeading())
//                .setBrakingStart(10)
//                .build();
//
//        parkRed = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPoseRed3, parkPoseRed)).setLinearHeadingInterpolation(pickupPoseRed3.getHeading(), parkPoseRed.getHeading())
//                .build();
//
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0: // start pose to score pose
//                runningActions.add(new InstantAction(() -> intake.intake()));
//                runTurret = true;
//                follower.followPath(blueAlliance ? startToScoreBlue : startToScoreRed, true);
//                setPathState(1);
//                break;
//            case 1: // spin up shooter
//                if (!follower.isBusy()) {
//                    runShooter = true;
//                    setPathState(2);
//                }
//                break;
//            case 2: // shooter up to speed, now shoot 3 balls
//                if (shooter.atTargetRPM && ballsLaunched < 3 && pathTimer.getElapsedTime() > TIME_BETWEEN_SHOTS) {
//                    runningActions.add(new SequentialAction(
//                            new InstantAction(() -> intake.intake()),
//                            new InstantAction(() -> shooter.openLatch()),
//                            new InstantAction(() -> intake.deployBallPusher()),
//                            new SleepAction(PUSHER_DELAY),
//                            new InstantAction(() -> intake.stowBallPusher()),
//                            new InstantAction(() -> shooter.closeLatch())
//                    ));
//                    ballsLaunched += 1;
//                    setPathState(2);
//                } else if (ballsLaunched == 3 && pathTimer.getElapsedTime() > 200) { // slow shooter, start intake, and go grab 3 more balls
//                    runShooter = false; // stops shooter + turret
//                    runTurret = false;
//                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
//                    follower.followPath(blueAlliance ? pickupBallsBlue1 : pickupBallsRed1,true);
//                    setPathState(3);
//                }
//                break;
//            case 3: // balls grabbed, now return to score
//                if(!follower.isBusy()) { //TODO: make this requirement looser, like x < scorePickup.getX() (blue and red)
//                    runningActions.add(new InstantAction(() -> intake.intake()));
//                    follower.followPath(blueAlliance ? scorePickupBlue1 : scorePickupRed1,true);
//                    setPathState(4);
//                }
//                break;
//            case 4: // spin up shooter
//                if (!follower.isBusy()) {
//                    runTurret = true;
//                    runShooter = true;
//                    setPathState(5);
//                }
//                break;
//            case 5: // shooter up to speed, now shoot next 3 balls
//                if (shooter.atTargetRPM && ballsLaunched < 6 && pathTimer.getElapsedTime() > TIME_BETWEEN_SHOTS) {
//                    runningActions.add(new SequentialAction(
//                            new InstantAction(() -> intake.intake()),
//                            new InstantAction(() -> shooter.openLatch()),
//                            new InstantAction(() -> intake.deployBallPusher()),
//                            new SleepAction(PUSHER_DELAY),
//                            new InstantAction(() -> intake.stowBallPusher()),
//                            new InstantAction(() -> shooter.closeLatch())
//                    ));
//                    ballsLaunched += 1;
//                    setPathState(5); // reset to self
//                } else if (ballsLaunched == 6 && pathTimer.getElapsedTime() > 200) {
//                    runShooter = false; // stops shooter + turret
//                    runTurret = false;
//                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
//                    follower.followPath(blueAlliance ? pickupBallsBlue2 : pickupBallsRed2,true);
//                    setPathState(6);
//                }
//                break;
//            case 6: // balls grabbed, now return to score
//                if(!follower.isBusy()) {
//                    runningActions.add(new InstantAction(() -> intake.intake()));
//                    follower.followPath(blueAlliance ? scorePickupBlue2 : scorePickupRed2,true);
//                    setPathState(7);
//                }
//                break;
//            case 7: // spin up shooter
//                if (!follower.isBusy()) {
//                    runShooter = true;
//                    runTurret = true;
//                    setPathState(8);
//                }
//                break;
//            case 8: // shooter up to speed, now shoot next 3 balls
//                if (shooter.atTargetRPM && ballsLaunched < 9 && pathTimer.getElapsedTime() > TIME_BETWEEN_SHOTS) {
//                    runningActions.add(new SequentialAction(
//                            new InstantAction(() -> intake.intake()),
//                            new InstantAction(() -> shooter.openLatch()),
//                            new InstantAction(() -> intake.deployBallPusher()),
//                            new SleepAction(PUSHER_DELAY),
//                            new InstantAction(() -> intake.stowBallPusher()),
//                            new InstantAction(() -> shooter.closeLatch())
//                    ));
//                    ballsLaunched += 1;
//                    setPathState(8); // reset to self
//                } else if (ballsLaunched == 9 && pathTimer.getElapsedTime() > 300) { // done, go park
//                    runShooter = false; // TODO: turret stays at angle, can make it spring back to center if we want
//                    runTurret = false;
//                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
//                    follower.followPath(blueAlliance ? pickupBallsBlue3: pickupBallsRed3, true);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                if (!follower.isBusy()) {
//                    follower.followPath(blueAlliance ? parkBlue: parkRed, true);
//                    runningActions.add(new InstantAction(() -> intake.intake()));
//                    setPathState(10);
//                }
//            case 10: // park
//                if(!follower.isBusy()) {
//                    runningActions.add(new SequentialAction(
//                            new InstantAction(() -> intake.zero()),
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
//    }
//
//    @Override
//    public void init_loop() {
//        if (gamepad1.b || gamepad2.b) {
//            blueAlliance = false;
//        } else if (gamepad1.x || gamepad2.x) {
//            blueAlliance = true;
//        }
//
//        telemetry.addLine("B for RED | X for BLUE");
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
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
//        llVision.trackPose(blueAlliance);
//        shooter.operateAutoLL(runShooter);
//        turret.operateAuto(runTurret, blueAlliance);
//        follower.update();
//        autonomousPathUpdate();
//
//        // Feedback to Driver Hub for debugging
//        telemetry.addData("current RPM ", shooter.getCurrentRPM());
//        telemetry.addData("target RPM", shooter.targetRPM);
//        telemetry.addData("atTargetRPM? ", shooter.atTargetRPM);
//        telemetry.addData("runShooter? ", runShooter);
//        telemetry.addData("looptimes ", elapsedtime.milliseconds());
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
//        telemetry.addLine("\n\n");
//        telemetry.addData("path state ", pathState);
//        telemetry.addData("isBusy? ", follower.isBusy());
//        telemetry.addData("x ", follower.getPose().getX());
//        telemetry.addData("y ", follower.getPose().getY());
//        telemetry.addData("heading ", follower.getPose().getHeading());
//        elapsedtime.reset();
//    }
//
//    @Override
//    public void stop() {
//        Globals.autoEndPose = follower.getPose();
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//}
