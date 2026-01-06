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
//@Autonomous(name = "Punahou 12 Ball Near", group = "A", preselectTeleOp = "Odom TeleOp")
//public class PunahouTwelveBallNear extends OpMode {
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
//    private final double PUSHER_DELAY = 0.5; // seconds (sleepAction0
//    private final double RAPID_FIRE_DELAY = 0.6; // seconds (sleepAction)
//    private final double DELAY_BEFORE_MOVING = 200; // milliseconds
//    private final double HOLD_GATE_DELAY = 500;
//
//    /// BLUE SIDE
//    private final Pose startPoseBlue = new Pose(25.4, 127.7, Math.toRadians(145));
//    private final Pose scorePoseBlue = new Pose(61, 84, Math.toRadians(180)); // 3rd score pose is 90°
//    private final Pose pickupPoseBlue1 = new Pose(23, 84, Math.toRadians(180));
//    private final Pose openGatePoseBlue = new Pose(24, 71, Math.toRadians(180));
//    private final Pose openGateControlPoseBlue = new Pose(40, 77, Math.toRadians(180));
//    private final Pose prepPickupPoseBlue2 = new Pose(51, 66, Math.toRadians(180));
//    private final Pose pickupPoseBlue2 = new Pose(21, 60, Math.toRadians(180));
//    private final Pose prepPickupPoseBlue3 = new Pose(58.5, 44.5, Math.toRadians(180));
//    private final Pose pickupPoseBlue3 = new Pose(21, 36, Math.toRadians(180));
//    private final Pose parkPoseBlue = new Pose(30, 72, Math.toRadians(180));
//
//    /// RED SIDE
//    private final Pose startPoseRed = startPoseBlue.mirror();
//    private final Pose scorePoseRed = new Pose(83, 84, Math.toRadians(0));
//    private final Pose pickupPoseRed1 = pickupPoseBlue1.mirror();
//    private final Pose openGatePoseRed = openGatePoseBlue.mirror();
//    private final Pose openGateControlPoseRed = openGateControlPoseBlue.mirror();
//    private final Pose prepPickupPoseRed2 = prepPickupPoseBlue2.mirror();
//    private final Pose pickupPoseRed2 = pickupPoseBlue2.mirror();
//    private final Pose prepPickupPoseRed3 = prepPickupPoseBlue3.mirror();
//    private final Pose pickupPoseRed3 = pickupPoseBlue3.mirror();
//    private final Pose parkPoseRed = parkPoseBlue.mirror();
//
//    private PathChain startToScoreBlue, pickupBallsBlue1, openGateBlue, scorePickupBlue1, pickupBallsBlue2, scorePickupBlue2, pickupBallsBlue3, scorePickupBlue3, parkBlue;
//    private PathChain startToScoreRed, pickupBallsRed1, openGateRed, scorePickupRed1, pickupBallsRed2, scorePickupRed2, pickupBallsRed3, scorePickupRed3, parkRed;
//
//    public void buildPaths() {
//        /// BLUE SIDE
//        startToScoreBlue = follower.pathBuilder()
//                .addPath(new BezierLine(startPoseBlue, scorePoseBlue)).setLinearHeadingInterpolation(startPoseBlue.getHeading(), scorePoseBlue.getHeading())
//                .build();
//
//        pickupBallsBlue1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseBlue, pickupPoseBlue1)).setLinearHeadingInterpolation(scorePoseBlue.getHeading(), pickupPoseBlue1.getHeading())
//                .setBrakingStart(10)
//                .build();
//
//        openGateBlue = follower.pathBuilder()
//                .addPath(new BezierCurve(pickupPoseBlue1, openGateControlPoseBlue, openGatePoseBlue)).setLinearHeadingInterpolation(pickupPoseBlue1.getHeading(), openGatePoseBlue.getHeading())
//                .setBrakingStart(14)
//                .build();
//
//        scorePickupBlue1 = follower.pathBuilder()
//                .addPath(new BezierLine(openGatePoseBlue, scorePoseBlue)).setLinearHeadingInterpolation(openGatePoseBlue.getHeading(), scorePoseBlue.getHeading())
//                .build();
//
//
//        pickupBallsBlue2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseBlue, prepPickupPoseBlue2)).setLinearHeadingInterpolation(scorePoseBlue.getHeading(), prepPickupPoseBlue2.getHeading())
//                .setBrakingStart(10)
//                .addPath(new BezierLine(prepPickupPoseBlue2, pickupPoseBlue2)).setLinearHeadingInterpolation(prepPickupPoseBlue2.getHeading(), pickupPoseBlue2.getHeading())
//                .setBrakingStart(14)
//                .build();
//
//        scorePickupBlue2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPoseBlue2, scorePoseBlue)).setLinearHeadingInterpolation(pickupPoseBlue2.getHeading(), scorePoseBlue.getHeading())
//                .build();
//
//        pickupBallsBlue3 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseBlue, prepPickupPoseBlue3)).setLinearHeadingInterpolation(scorePoseBlue.getHeading(), prepPickupPoseBlue3.getHeading())
//                .setBrakingStart(15)
//                .addPath(new BezierLine(prepPickupPoseBlue3, pickupPoseBlue3)).setLinearHeadingInterpolation(prepPickupPoseBlue3.getHeading(), pickupPoseBlue3.getHeading(), 0.3)
//                .setBrakingStart(15)
//                .build();
//
//        scorePickupBlue3 = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPoseBlue3, scorePoseBlue)).setLinearHeadingInterpolation(pickupPoseBlue3.getHeading(), scorePoseBlue.getHeading())
//                .build();
//
//        parkBlue = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseBlue, parkPoseBlue)).setLinearHeadingInterpolation(scorePoseBlue.getHeading(), parkPoseBlue.getHeading())
//                .build();
//
//        /// RED SIDE
//        startToScoreRed = follower.pathBuilder()
//                .addPath(new BezierLine(startPoseRed, scorePoseRed)).setLinearHeadingInterpolation(startPoseRed.getHeading(), scorePoseRed.getHeading())
//                .build();
//
//        pickupBallsRed1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseRed, pickupPoseRed1)).setLinearHeadingInterpolation(scorePoseRed.getHeading(), pickupPoseRed1.getHeading())
//                .setBrakingStart(10)
//                .build();
//
//        openGateRed = follower.pathBuilder()
//                .addPath(new BezierCurve(pickupPoseRed1, openGateControlPoseRed, openGatePoseRed)).setLinearHeadingInterpolation(pickupPoseRed1.getHeading(), openGatePoseRed.getHeading())
//                .setBrakingStart(14)
//                .build();
//
//        scorePickupRed1 = follower.pathBuilder()
//                .addPath(new BezierLine(openGatePoseRed, scorePoseRed)).setLinearHeadingInterpolation(openGatePoseRed.getHeading(), scorePoseRed.getHeading())
//                .build();
//
//        pickupBallsRed2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseRed, prepPickupPoseRed2)).setLinearHeadingInterpolation(scorePoseRed.getHeading(), prepPickupPoseRed2.getHeading())
//                .setBrakingStart(10)
//                .addPath(new BezierLine(prepPickupPoseRed2, pickupPoseRed2)).setLinearHeadingInterpolation(prepPickupPoseRed2.getHeading(), pickupPoseRed2.getHeading())
//                .setBrakingStart(14)
//                .build();
//
//        scorePickupRed2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPoseRed2, scorePoseRed)).setLinearHeadingInterpolation(pickupPoseRed2.getHeading(), scorePoseRed.getHeading())
//                .build();
//
//        pickupBallsRed3 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseRed, prepPickupPoseRed3)).setLinearHeadingInterpolation(scorePoseRed.getHeading(), prepPickupPoseRed3.getHeading())
//                .setBrakingStart(15)
//                .addPath(new BezierLine(prepPickupPoseRed3, pickupPoseRed3)).setLinearHeadingInterpolation(prepPickupPoseRed3.getHeading(), pickupPoseRed3.getHeading(), 0.3)
//                .setBrakingStart(15)
//                .build();
//
//        scorePickupRed3 = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPoseRed3, scorePoseRed)).setLinearHeadingInterpolation(pickupPoseRed3.getHeading(), scorePoseRed.getHeading())
//                .build();
//
//        parkRed = follower.pathBuilder()
//                .addPath(new BezierLine(scorePoseRed, parkPoseRed)).setLinearHeadingInterpolation(scorePoseRed.getHeading(), parkPoseRed.getHeading())
//                .build();
//
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            /// BALLS 1, 2, 3
//            case 0: // start pose to score pose
//                runningActions.add(new InstantAction(() -> intake.intake()));
//                runTurret = true;
//                follower.followPath(blueAlliance ? startToScoreBlue : startToScoreRed, true);
//                setPathState(1);
//                break;
//            case 1: // spin up shooter separately to avoid brownout
//                if (!follower.isBusy()) {
//                    runShooter = true;
//                    setPathState(2);
//                }
//                break;
//            case 2: // shooter up to speed, now shoot 3 balls
//                if (shooter.atTargetRPM && llVision.tagDetected(blueAlliance) && pathTimer.getElapsedTime() > 1000) {
//                    rapidFireAction();
//                    setPathState(3);
//                }
//                break;
//            case 3: // done rapid-firing, pick up next balls
//                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
//                    // slow shooter, start intake, and go grab 3 more balls
//                    ballsLaunched = 3; // kinda deprecated now
//                    runShooter = false; // stops shooter + turret
//                    runTurret = false;
//                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
//                    follower.followPath(blueAlliance ? pickupBallsBlue1 : pickupBallsRed1,true);
//                    setPathState(4);
//                }
//                break;
//            /// OPEN GATE
//            case 4: // balls grabbed, now open gate
//                if (!follower.isBusy()) {
//                    runningActions.add(new InstantAction(() -> intake.intake()));
//                    follower.followPath(blueAlliance ? openGateBlue : openGateRed);
//                    setPathState(5);
//                }
//                break;
//            case 5: // opened gate
//                if (!follower.isBusy()) {
//                    setPathState(50);
//                }
//                break;
//            case 50: // short delay to let balls out, then return to score
//                if (pathTimer.getElapsedTime() > HOLD_GATE_DELAY) {
//                    follower.followPath(blueAlliance ? scorePickupBlue1 : scorePickupRed1,true);
//                    setPathState(6);
//                    }
//                break;
//            ///  BALLS 4, 5, 6
//            case 6: // spin up shooter
//                if (!follower.isBusy()) {
//                    runTurret = true;
//                    runShooter = true;
//                    setPathState(7);
//                }
//                break;
//            case 7: // shooter up to speed, now shoot 3 balls
//                if (shooter.atTargetRPM && llVision.tagDetected(blueAlliance)) {
//                    rapidFireAction();
//                    setPathState(8);
//                }
//                break;
//            case 8: // done rapid-firing, pick up next balls
//                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
//                    // slow shooter, start intake, and go grab 3 more balls
//                    ballsLaunched = 6;
//                    runShooter = false; // stops shooter + turret
//                    runTurret = false;
//                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
//                    follower.followPath(blueAlliance ? pickupBallsBlue2 : pickupBallsRed2,true);
//                    setPathState(9);
//                }
//                break;
//            ///  BALLS 7, 8, 9
//            case 9: // balls grabbed, now return to score
//                if(!follower.isBusy()) {
//                    runningActions.add(new InstantAction(() -> intake.intake()));
//                    follower.followPath(blueAlliance ? scorePickupBlue2 : scorePickupRed2,true);
//                    setPathState(10);
//                }
//                break;
//            case 10: // spin up shooter
//                if (!follower.isBusy()) {
//                    runShooter = true;
//                    runTurret = true;
//                    setPathState(11);
//                }
//                break;
//            case 11: // shooter up to speed, now shoot next 3 balls
//                if (shooter.atTargetRPM && llVision.tagDetected(blueAlliance)) {
//                    rapidFireAction();
//                    setPathState(12);
//                }
//                break;
//            case 12: // done rapid-firing, pick up next balls
//                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
//                    // slow shooter, start intake, and go grab 3 more balls
//                    ballsLaunched = 9;
//                    runShooter = false; // stops shooter + turret
//                    runTurret = false;
//                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
//                    follower.followPath(blueAlliance ? pickupBallsBlue3 : pickupBallsRed3,true);
//                    setPathState(13);
//                }
//                break;
//            ///  BALLS 10, 11, 12
//            case 13: // balls grabbed, now return to score
//                if (!follower.isBusy()) {
////                    runTurret = true;
//                    follower.followPath(blueAlliance ? scorePickupBlue3: scorePickupRed3, true);
//                    runningActions.add(new InstantAction(() -> intake.intake()));
//                    setPathState(14);
//                }
//                break;
//            case 14: // spin up shooter
//                if (!follower.isBusy()) {
//                    runShooter = true;
//                    runTurret = true; // duplicate, but keep for now
//                    setPathState(15);
//                }
//                break;
//            case 15: // shooter up to speed, now shoot next 3 balls
//                if (shooter.atTargetRPM && llVision.tagDetected(blueAlliance)) {
//                    rapidFireAction();
//                    setPathState(16);
//                }
//                break;
//            /// PARK
//            case 16: // done rapid-firing, now park
//                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
//                    runShooter = false;
//                    runTurret = false;
//                    ballsLaunched = 12;
//                    follower.followPath(blueAlliance ? parkBlue: parkRed, true);
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
//   //     lights.setColor(blueAlliance ? RGBLights.Colors.BLUE : RGBLights.Colors.RED);
//
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
//        llVision.trackPose(blueAlliance);
//        shooter.operateAutoLL(runShooter);
//        turret.operateAutoLL(runTurret, blueAlliance);
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
//                new InstantAction(() -> pathTimer.resetTimer()), // kinda weird, but necessary so robot waits for a moment after scoring
//                new InstantAction(() -> intake.stowBallPusher()),
//                new InstantAction(() -> shooter.closeLatch()),
//                new InstantAction(() -> intake.idle())
//        ));
//    }
//}
