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
//import org.firstinspires.ftc.teamcode.Util.PinpointManager;
//import org.firstinspires.ftc.teamcode.Util.RobotHardware;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.Util.RGBLights;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@Autonomous(name = "9 Ball Far", group = "A", preselectTeleOp = "Odom TeleOp")
//public class NineBallFar extends OpMode {
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
//    private PinpointManager pinpoint = new PinpointManager(); // IMPORTANT
//    private Intake intake = new Intake();
//    private Turret turret = new Turret();
//    private FlywheelShooter shooter = new FlywheelShooter();
//    private LimelightVision llVision = new LimelightVision();
//    private RGBLights lights = new RGBLights();
//
//
//    ///  CONSTANTS
//    private boolean blueAlliance = true;
//    private int numCycles = 0;
//    private int totalCycles = 3; // 3 hp cycles for now -> can adjust later
//    private volatile boolean runShooter = false;
//    private volatile boolean runTurret = false;
//    private final double PUSHER_DELAY = 0.5; // seconds (sleepAction)
//    private final double RAPID_FIRE_DELAY = 0.6; // seconds (sleepAction)
//    private final double DELAY_BEFORE_MOVING = 200; // milliseconds
//    private final double  DELAY_BEFORE_SHOOTING = 2000; // adjust based on cyber's auto
//    private boolean cyclingFarZone = true; // for shooter
//
//    /// BLUE SIDE
//    private final Pose farStartPoseBlue = new Pose(54, 6, Math.toRadians(180));
//    private final Pose farScorePoseBlue = new Pose(58, 10, Math.toRadians(180)); // after collecting first and second sets of balls
//    // private final Pose prepPickupPoseBlue = new Pose(50, 35, Math.toRadians(180));
//    // private final Pose farPickupPoseBlue = new Pose(18, 35, Math.toRadians(180));
//    private final Pose hpPickupPoseBlue1 = new Pose(15, 16, Math.toRadians(220));
//    private final Pose hpPickupPoseBlue2 = new Pose(15, 11, Math.toRadians(220));
//    private final Pose hpPickupPoseBlue3 = new Pose(13, 7, Math.toRadians(180));
//    private final Pose hpPickupPoseBlue4 = new Pose(10, 7, Math.toRadians(180));
//    private final Pose farParkPoseBlue = new Pose(38, 10, Math.toRadians(180));
//
//    // another potential set of poses that could work for hp pickup:
//        // private final Pose hpPickupPoseBlue1 = new Pose(12.5, 18, Math.toRadians(220));
//        // private final Pose hpPickupPoseBlue2 = new Pose(12.5, 12, Math.toRadians(220));
//        // private final Pose hpPickupPoseBlue3 = new Pose(12.5, 10, Math.toRadians(180));
//
//    /// RED SIDE
//    private final Pose farStartPoseRed = farStartPoseBlue.mirror();
//    private final Pose farScorePoseRed = farScorePoseBlue.mirror();
//    // private final Pose prepPickupPoseRed = prepPickupPoseBlue.mirror();
//    // private final Pose farPickupPoseRed = farPickupPoseBlue.mirror();
//    private final Pose hpPickupPoseRed1 = hpPickupPoseBlue1.mirror();
//
//    private final Pose hpPickupPoseRed2 = hpPickupPoseBlue2.mirror();
//    private final Pose hpPickupPoseRed3 = hpPickupPoseBlue3.mirror();
//    private final Pose hpPickupPoseRed4 = hpPickupPoseBlue4.mirror();
//    private final Pose farParkPoseRed = farParkPoseBlue.mirror();
//
//    // path chains:
//    private PathChain pickupBallsBlue1, scorePickupBlue1, pickupBallsBlue2, scorePickupBlue2, parkBlue;
//    private PathChain pickupBallsRed1, scorePickupRed1, pickupBallsRed2, scorePickupRed2, parkRed;
//
//    public void buildPaths() {
//        /// BLUE SIDE
//
//        /* pickupBallsBlue1 = follower.pathBuilder()
//                .addPath(new BezierLine(farStartPoseBlue, prepPickupPoseBlue)).setLinearHeadingInterpolation(farStartPoseBlue.getHeading(), prepPickupPoseBlue.getHeading())
//                .addPath(new BezierLine(prepPickupPoseBlue, farPickupPoseBlue)).setLinearHeadingInterpolation(prepPickupPoseBlue.getHeading(), farPickupPoseBlue.getHeading())
//                .setBrakingStart(10)
//                .build();
//
//        scorePickupBlue1 = follower.pathBuilder()
//                .addPath(new BezierLine(farPickupPoseBlue, farScorePoseBlue)).setLinearHeadingInterpolation(farPickupPoseBlue.getHeading(), farScorePoseBlue.getHeading())
//                .build();
//
//         */
//
//        pickupBallsBlue1 = follower.pathBuilder()
//                .addPath(new BezierLine(farStartPoseBlue, hpPickupPoseBlue1)).setLinearHeadingInterpolation(farStartPoseBlue.getHeading(), hpPickupPoseBlue1.getHeading())
//                .addPath(new BezierLine(hpPickupPoseBlue1, hpPickupPoseBlue2)).setLinearHeadingInterpolation(hpPickupPoseBlue1.getHeading(), hpPickupPoseBlue2.getHeading())
//                .addPath(new BezierLine(hpPickupPoseBlue2, hpPickupPoseBlue3)).setLinearHeadingInterpolation(hpPickupPoseBlue2.getHeading(), hpPickupPoseBlue3.getHeading())
//                .setBrakingStart(10)
//                .build();
//
//        scorePickupBlue1 = follower.pathBuilder()
//                .addPath(new BezierLine(hpPickupPoseBlue3, hpPickupPoseBlue4)).setLinearHeadingInterpolation(hpPickupPoseBlue3.getHeading(), hpPickupPoseBlue4.getHeading())
//                .addPath(new BezierLine(hpPickupPoseBlue4, farScorePoseBlue)).setLinearHeadingInterpolation(hpPickupPoseBlue4.getHeading(), farScorePoseBlue.getHeading())
//                .build();
//
//        pickupBallsBlue2 = follower.pathBuilder()
//                .addPath(new BezierLine(farScorePoseBlue, hpPickupPoseBlue1)).setLinearHeadingInterpolation(farScorePoseBlue.getHeading(), hpPickupPoseBlue1.getHeading())
//                .addPath(new BezierLine(hpPickupPoseBlue1, hpPickupPoseBlue2)).setLinearHeadingInterpolation(hpPickupPoseBlue1.getHeading(), hpPickupPoseBlue2.getHeading())
//                .addPath(new BezierLine(hpPickupPoseBlue2, hpPickupPoseBlue3)).setLinearHeadingInterpolation(hpPickupPoseBlue2.getHeading(), hpPickupPoseBlue3.getHeading())
//                .setBrakingStart(10)
//                .build();
//
//        scorePickupBlue2 = follower.pathBuilder()
//                .addPath(new BezierLine(hpPickupPoseBlue3, hpPickupPoseBlue4)).setLinearHeadingInterpolation(hpPickupPoseBlue3.getHeading(), hpPickupPoseBlue4.getHeading())
//                .addPath(new BezierLine(hpPickupPoseBlue4, farScorePoseBlue)).setLinearHeadingInterpolation(hpPickupPoseBlue4.getHeading(), farScorePoseBlue.getHeading())
//                .build();
//
//        parkBlue = follower.pathBuilder()
//                .addPath(new BezierLine(farScorePoseBlue, farParkPoseBlue)).setLinearHeadingInterpolation(farScorePoseBlue.getHeading(), farParkPoseBlue.getHeading())
//                .build();
//
//
//
//        /// RED SIDE
//
//        /* pickupBallsRed1 = follower.pathBuilder()
//                .addPath(new BezierLine(farStartPoseRed, prepPickupPoseRed)).setLinearHeadingInterpolation(farStartPoseRed.getHeading(), prepPickupPoseRed.getHeading())
//                .addPath(new BezierLine(prepPickupPoseRed, farPickupPoseRed)).setLinearHeadingInterpolation(prepPickupPoseRed.getHeading(), farPickupPoseRed.getHeading())
//                .setBrakingStart(10)
//                .build();
//
//        scorePickupRed1 = follower.pathBuilder()
//                .addPath(new BezierLine(farPickupPoseRed, farScorePoseRed)).setLinearHeadingInterpolation(farPickupPoseRed.getHeading(), farScorePoseRed.getHeading())
//                .build();
//
//         */
//
//        pickupBallsRed1 = follower.pathBuilder()
//                .addPath(new BezierLine(farStartPoseRed, hpPickupPoseRed1)).setLinearHeadingInterpolation(farStartPoseRed.getHeading(), hpPickupPoseRed1.getHeading())
//                .addPath(new BezierLine(hpPickupPoseRed1, hpPickupPoseRed2)).setLinearHeadingInterpolation(hpPickupPoseRed1.getHeading(), hpPickupPoseRed2.getHeading())
//                .addPath(new BezierLine(hpPickupPoseRed2, hpPickupPoseRed3)).setLinearHeadingInterpolation(hpPickupPoseRed2.getHeading(), hpPickupPoseRed3.getHeading())
//                .addPath(new BezierLine(hpPickupPoseRed3, hpPickupPoseRed4)).setLinearHeadingInterpolation(hpPickupPoseRed3.getHeading(), hpPickupPoseRed4.getHeading())
//                .setBrakingStart(10)
//                .build();
//
//        scorePickupRed1 = follower.pathBuilder()
//                .addPath(new BezierLine(hpPickupPoseRed4, farScorePoseRed)).setLinearHeadingInterpolation(hpPickupPoseRed4.getHeading(), farScorePoseRed.getHeading())
//                .build();
//
//        pickupBallsRed2 = follower.pathBuilder()
//                .addPath(new BezierLine(farScorePoseRed, hpPickupPoseRed1)).setLinearHeadingInterpolation(farScorePoseRed.getHeading(), hpPickupPoseRed1.getHeading())
//                .addPath(new BezierLine(hpPickupPoseRed1, hpPickupPoseRed2)).setLinearHeadingInterpolation(hpPickupPoseRed1.getHeading(), hpPickupPoseRed2.getHeading())
//                .addPath(new BezierLine(hpPickupPoseRed2, hpPickupPoseRed3)).setLinearHeadingInterpolation(hpPickupPoseRed2.getHeading(), hpPickupPoseRed3.getHeading())
//                .addPath(new BezierLine(hpPickupPoseRed3, hpPickupPoseRed4)).setLinearHeadingInterpolation(hpPickupPoseRed3.getHeading(), hpPickupPoseRed4.getHeading())
//                .setBrakingStart(10)
//                .build();
//
//        scorePickupRed2 = follower.pathBuilder()
//                .addPath(new BezierLine(hpPickupPoseRed4, farScorePoseRed)).setLinearHeadingInterpolation(hpPickupPoseRed4.getHeading(), farScorePoseRed.getHeading())
//                .build();
//
//        parkRed = follower.pathBuilder()
//                .addPath(new BezierLine(farScorePoseRed, farParkPoseRed)).setLinearHeadingInterpolation(farScorePoseRed.getHeading(), farParkPoseRed.getHeading())
//                .build();
//
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            /// BALLS 1, 2, 3
//            case 0: // spin up shooter + set turret
//                if (shooter.getCurrentRPM() > 2000) {
//                    runShooter = true;
//                    runTurret = true;
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
//                    // slow shooter, start intake, and go grab 3 more balls
//                    // no num cycles update bc shooting preload
//                    runShooter = false;
//                    runTurret =  false;
//                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
//                    follower.followPath(blueAlliance ? pickupBallsBlue1 : pickupBallsRed1,0.6, false);
//                    setPathState(4);
//                }
//                break;
//            case 4: // go back to score at the far position
//                if (!follower.isBusy() || follower.isRobotStuck() || pathTimer.getElapsedTime() > 2500) {
//                    follower.followPath(blueAlliance ? scorePickupBlue1 : scorePickupRed1,true);
//                    setPathState(5);
//                }
//                break;
//            ///  BALLS 4, 5, 6
//            case 5: // spin up shooter
//                if (!follower.isBusy() || follower.isRobotStuck()) {
//                    runShooter = true;
//                    runTurret = true;
//                    setPathState(6);
//                }
//                break;
//            case 6: // shooter up to speed, now shoot 3 balls
//                if (shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen  && pathTimer.getElapsedTime() > DELAY_BEFORE_SHOOTING) {
//                    rapidFireAction();
//                    setPathState(7);
//                }
//                break;
//            case 7: // done rapid-firing, pick up next balls
//                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
//                    // slow shooter, start intake, and go grab 3 more balls
//                    numCycles++;
//                    runShooter = false;
//                    runTurret = false;
//                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                follower.followPath(blueAlliance ? pickupBallsBlue2 : pickupBallsRed2,0.6,false);
//                runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
//                setPathState(9);
//                break;
//            ///  BALLS 7, 8, 9
//            case 9: // balls grabbed, now return to score
//                if(!follower.isBusy() || follower.isRobotStuck() || pathTimer.getElapsedTime() > 2500) {
//                    runningActions.add(new InstantAction(() -> intake.intake()));
//                    follower.followPath(blueAlliance ? scorePickupBlue2 : scorePickupRed2, true);
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
//                if (shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_SHOOTING) {
//                    rapidFireAction();
//                    setPathState(12);
//                }
//                break;
//            case 12: // done rapid-firing, park
//                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
//                    numCycles++;
//                    runShooter = false;
//                    runTurret = false;
//                    if (numCycles < totalCycles)
//                        setPathState(8);
//                    else
//                        setPathState(14);
//                }
//                break;
//            case 14:
//                follower.followPath(blueAlliance ? parkBlue : parkRed,true);
//                setPathState(-1);
//
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
//        pinpoint.initialize(this, robotHardware);
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
//        follower.setStartingPose(blueAlliance ? farStartPoseBlue : farStartPoseRed);
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
//        // odom tracking loops
//        shooter.operateOdomAuto(follower.getPose().getX(), follower.getPose().getY(), blueAlliance, runShooter, cyclingFarZone);
//        turret.operateOdomAuto(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()), blueAlliance, runTurret);
//
//        // other loops
//        intake.operateCurrentLimiting();
//        follower.update();
//        autonomousPathUpdate();
//
//        // Feedback to Driver Hub for debugging
//        telemetry.addLine("STATE TELEMETRY");
//        telemetry.addData("path state ", pathState);
//        telemetry.addData("runShooter? ", runShooter);
//        telemetry.addData("runTurret? ", runTurret);
//        telemetry.addData("cycles ", numCycles);
//        telemetry.addLine("\n");
//        telemetry.addLine("PEDRO TELEMETRY");
//        telemetry.addData("isBusy? ", follower.isBusy());
//        telemetry.addData("x ", follower.getPose().getX());
//        telemetry.addData("y ", follower.getPose().getY());
//        telemetry.addData("heading ", Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.addLine("\n");
//        telemetry.addData("looptimes ", elapsedtime.milliseconds());
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
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
//                new SleepAction(0.05),
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
