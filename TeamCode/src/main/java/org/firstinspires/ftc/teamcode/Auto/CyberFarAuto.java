package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
import org.firstinspires.ftc.teamcode.Util.PinpointManager;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Util.RGBLights;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Cyber Far Auto", group = "A", preselectTeleOp = "Odom TeleOp")
public class CyberFarAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private List<LynxModule> allHubs;
    private ElapsedTime elapsedtime;


    private RobotHardware robotHardware = new RobotHardware();
    private PinpointManager pinpoint = new PinpointManager(); // IMPORTANT
    private Intake intake = new Intake();
    private Turret turret = new Turret();
    private FlywheelShooter shooter = new FlywheelShooter();
    private LimelightVision llVision = new LimelightVision();
    private BrakePad brake = new BrakePad();
    private RGBLights lights = new RGBLights();


    ///  STATE VARIABLES
    private boolean blueAlliance = true;
    private int numCycles = 0;
    private volatile boolean runShooter = false;
    private volatile boolean runTurret = false;
    private boolean cyclingFarZone = true; // for shooter

    ///  CONSTANTS
    private final int totalCycles = 3; // number of cycles after balls 789
    private final double PUSHER_DELAY = 0.5; // seconds (sleepAction)
    private final double RAPID_FIRE_DELAY = 0.6; // seconds (sleepAction)
    private final double DELAY_BEFORE_MOVING = 150; // milliseconds
    private final double ROBOT_STUCK_THRESHOLD = 2000;
    private final double SLOW_BRAKE_STRENGTH = 1;

    /// BLUE SIDE
    /// field centered at x=70.5
    private final Pose startPoseBlue = new Pose(56, 8, Math.toRadians(180)); // same as score 123 pose
    private final Pose waitPickup456PoseBlue = new Pose(8.5, 27, Math.toRadians(100));
    private final Pose scorePoseBlue456 = new Pose(48, 9, Math.toRadians(160));
    private final Pose waitPickup789PoseBlue = new Pose(9, 36, Math.toRadians(100));
    private final double WAIT_AT_HP_789 = 1500; // TODO shorten if can
    private final Pose scorePoseBlueCycles = new Pose(48, 9, Math.toRadians(170));
    private final Pose hpPickupPoseBlue1 = new Pose(18, 10, Math.toRadians(180)); // TODO tune for 10, 11, 12, 13, 14, 15, etc
    private final Pose hpPickupPoseBlue2 = new Pose(19, 10, Math.toRadians(160)); // TODO tune
    private final Pose hpPickupPoseBlue3 = new Pose(15, 13, Math.toRadians(170)); // TODO tune
    private final Pose parkPoseBlue = new Pose(44, 12, Math.toRadians(135));

    // another potential set of poses that could work for hp pickup:
    // private final Pose hpPickupPoseBlue1 = new Pose(12.5, 18, Math.toRadians(220));
    // private final Pose hpPickupPoseBlue2 = new Pose(12.5, 12, Math.toRadians(220));
    // private final Pose hpPickupPoseBlue3 = new Pose(12.5, 10, Math.toRadians(180));

    /// RED SIDE
    private final Pose startPoseRed = startPoseBlue.mirror(); // same as score 123 pose
    private final Pose waitPickup456PoseRed = waitPickup456PoseBlue.mirror();
    private final Pose scorePoseRed456 = scorePoseBlue456.mirror();
    private final Pose waitPickup789PoseRed = waitPickup789PoseBlue.mirror();
    private final Pose scorePoseRedCycles = scorePoseBlueCycles.mirror();
    private final Pose hpPickupPoseRed1 = hpPickupPoseBlue1.mirror(); // for 10, 11, 12, 13, 14, 15, etc
    private final Pose hpPickupPoseRed2 = hpPickupPoseBlue2.mirror();
    private final Pose hpPickupPoseRed3 = hpPickupPoseBlue3.mirror();
    private final Pose parkPoseRed = parkPoseBlue.mirror();

    // path chains:
    private PathChain pickupBallsBlue789, scorePickupBlue789, pickupBallsBlueCycles, scorePickupBlueCycles, parkBlue;
    private PathChain pickupBallsRed789, scorePickupRed789, pickupBallsRedCycles, scorePickupRedCycles, parkRed;

    public void buildPaths() {
        /// BLUE SIDE

        pickupBallsBlue789 = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseBlue456, waitPickup789PoseBlue)).setLinearHeadingInterpolation(scorePoseBlue456.getHeading(), waitPickup789PoseBlue.getHeading())
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

        scorePickupBlue789 = follower.pathBuilder()
                .addPath(new BezierLine(waitPickup789PoseBlue, scorePoseBlueCycles)).setLinearHeadingInterpolation(waitPickup789PoseBlue.getHeading(), scorePoseBlueCycles.getHeading())
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

        pickupBallsBlueCycles = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseBlueCycles, hpPickupPoseBlue1)).setLinearHeadingInterpolation(scorePoseBlueCycles.getHeading(), hpPickupPoseBlue1.getHeading())
                .addPath(new BezierLine(hpPickupPoseBlue1, hpPickupPoseBlue2)).setLinearHeadingInterpolation(hpPickupPoseBlue1.getHeading(), hpPickupPoseBlue2.getHeading())
                .addPath(new BezierLine(hpPickupPoseBlue2, hpPickupPoseBlue3)).setLinearHeadingInterpolation(hpPickupPoseBlue2.getHeading(), hpPickupPoseBlue3.getHeading())
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

        scorePickupBlueCycles = follower.pathBuilder()
                .addPath(new BezierLine(hpPickupPoseBlue3, scorePoseBlueCycles)).setLinearHeadingInterpolation(hpPickupPoseBlue3.getHeading(), scorePoseBlueCycles.getHeading())
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

        parkBlue = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseBlueCycles, parkPoseBlue)).setLinearHeadingInterpolation(scorePoseBlueCycles.getHeading(), parkPoseBlue.getHeading())
                .build();

        /// RED SIDE

        pickupBallsRed789 = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseRed456, waitPickup789PoseRed)).setLinearHeadingInterpolation(scorePoseRed456.getHeading(), waitPickup789PoseRed.getHeading())
                .build();

        scorePickupRed789 = follower.pathBuilder()
                .addPath(new BezierLine(waitPickup789PoseRed, scorePoseRedCycles)).setLinearHeadingInterpolation(waitPickup789PoseRed.getHeading(), scorePoseRedCycles.getHeading())
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

        pickupBallsRedCycles = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseRedCycles, hpPickupPoseRed1)).setLinearHeadingInterpolation(scorePoseRedCycles.getHeading(), hpPickupPoseRed1.getHeading())
                .addPath(new BezierLine(hpPickupPoseRed1, hpPickupPoseRed2)).setLinearHeadingInterpolation(hpPickupPoseRed1.getHeading(), hpPickupPoseRed2.getHeading())
                .addPath(new BezierLine(hpPickupPoseRed2, hpPickupPoseRed3)).setLinearHeadingInterpolation(hpPickupPoseRed2.getHeading(), hpPickupPoseRed3.getHeading())
                .build();

        scorePickupRedCycles = follower.pathBuilder()
                .addPath(new BezierLine(hpPickupPoseRed3, scorePoseRedCycles)).setLinearHeadingInterpolation(hpPickupPoseRed3.getHeading(), scorePoseRedCycles.getHeading())
                .setGlobalDeceleration(SLOW_BRAKE_STRENGTH)
                .build();

        parkRed = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseRedCycles, parkPoseRed)).setLinearHeadingInterpolation(scorePoseRedCycles.getHeading(), parkPoseRed.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            /// BALLS 1, 2, 3
            case 0: // wait, to spin up shooter, then set turret
                if (shooter.getCurrentRPM() > 2000) {
                    runShooter = true;
                    runTurret = true;
                    setPathState(1);
                }
                break;
            case 1: // shooter up to speed, now shoot 3 balls
                if (shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
                    rapidFireAction();
                    setPathState(2);
                }
                break;
            case 2: // done rapid-firing, pick up next balls
                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    // slow shooter, start intake, and go grab 3 more balls
                    // no num cycles update bc shooting preload
                    runShooter = false;
                    runTurret = false;
                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
                    follower.followPath(blueAlliance ? pickupBallsBlueCycles : pickupBallsRedCycles, true);
                    setPathState(4);
                }
                break;
            ///  BALLS 4, 5, 6
            case 4: // wait until gate opened and balls picked up, then go to score
                if (!follower.isBusy()) {
                    delayedIdle();
                    follower.followPath(blueAlliance ? scorePickupBlueCycles : scorePickupRedCycles, true);
                    setPathState(5);
                }
                break;
            case 5: // spin up shooter
                if (!follower.isBusy()) {
                    runShooter = true;
                    runTurret = true;
                    setPathState(6);
                }
                break;
            case 6: // shooter up to speed, now shoot 3 balls
                if (shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
                    rapidFireAction();
                    setPathState(7);
                }
                break;
            case 7: // done rapid-firing, wait at HP zone to grab balls
                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    runTurret = false;
                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
                    follower.followPath(blueAlliance ? pickupBallsBlueCycles : pickupBallsRedCycles, true);
                    setPathState(9);
                }
                break;
            ///  BALLS 7, 8, 9
            case 9: // wait until gate opened and balls picked up, then go to score
                if (!follower.isBusy()) {
                    delayedIdle();
                    follower.followPath(blueAlliance ? scorePickupBlueCycles : scorePickupRedCycles, true);
                    setPathState(10);
                }
                break;
            case 10: // spin up shooter
                if (!follower.isBusy()) {
                    runShooter = true;
                    runTurret = true;
                    setPathState(11);
                }
                break;
            case 11: // shooter up to speed, now shoot
                if (shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
                    rapidFireAction();
                    setPathState(12);
                }
                break;
            case 12: // done rapid-firing, grab balls from HP zone
                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    runTurret = false;
                    runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
                    follower.followPath(blueAlliance ? pickupBallsBlue789 : pickupBallsRed789, true);
                    setPathState(130);
                }
                break;
            ///  10, 11, 12
            case 130:
                if (!follower.isBusy()) {
                    setPathState(140);
                }
                break;
            case 140: // wait until gate opened and balls picked up, then go to score
                if (pathTimer.getElapsedTime() > WAIT_AT_HP_789) {
                    delayedIdle();
                    follower.followPath(blueAlliance ? scorePickupBlue789 : scorePickupRed789, true);
                    setPathState(150);
                }
                break;
            case 150: // spin up shooter
                if (!follower.isBusy()) {
                    runShooter = true;
                    runTurret = true;
                    setPathState(160);
                }
                break;
            case 160: // shooter up to speed, now shoot
                if (shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
                    rapidFireAction();
                    setPathState(170);
                }
                break;
            case 170: // done rapid-firing, grab balls from HP zone
                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    runTurret = false;
                    setPathState(13);
                }
                break;
            /// CYCLING HP
            /// CYCLING HP
            /// CYCLING HP
            case 13: // grab balls
                runningActions.add(new InstantAction(() -> intake.intakeFullPower()));
                follower.followPath(blueAlliance ? pickupBallsBlueCycles : pickupBallsRedCycles, false);
                setPathState(14);
                break;
            case 14: // balls grabbed, now return to score
                if (!follower.isBusy() || follower.isRobotStuck() || pathTimer.getElapsedTime() > ROBOT_STUCK_THRESHOLD) {
                    delayedIdle();
                    follower.followPath(blueAlliance ? scorePickupBlueCycles : scorePickupRedCycles, true);
                    setPathState(15);
                }
                break;
            case 15: // spin up shooter
                if (!follower.isBusy()) {
                    runShooter = true;
                    runTurret = true;
                    setPathState(16);
                }
                break;
            case 16: // shooter up to speed, now shoot next 3 balls
                if (shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
                    rapidFireAction();
                    setPathState(17);
                }
                break;
            case 17: // done shooting, next cycle or park
                if (!shooter.shooterLatchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    numCycles++;
                    runShooter = false;
                    runTurret = false;
                    if (opmodeTimer.getElapsedTimeSeconds() > 25) { // not enough time for another cycle, just park
                        setPathState(100);
                    }
                    else if (numCycles < totalCycles) { // still some cycles left to go
                        setPathState(13);
                    }
                    else { // final catch-all park
                        setPathState(100);
                    }
                }
                break;
            case 100: // park
                runShooter = false; // back to idle, will turn off when opmode stops
                runTurret = false; // return to center
                runningActions.add(new SequentialAction( // technically redundant, but doesn't hurt to have
                        new InstantAction(() -> intake.idle()),
                        new InstantAction(() -> intake.stowBallPusher()),
                        new InstantAction(() -> shooter.closeLatch())
                ));
                follower.followPath(blueAlliance ? parkBlue : parkRed,true);
                setPathState(-1);
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
        pinpoint.initialize(this, robotHardware);
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
        telemetry.addData("heading ", follower.getPose().getHeading());
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

        // odom tracking loops
        shooter.operateOdomAuto(follower.getPose().getX(), follower.getPose().getY(), blueAlliance, runShooter, cyclingFarZone);
        turret.operateOdomAuto(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()), blueAlliance, runTurret);

        // other loops
        intake.operateCurrentLimiting();
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addLine("STATE TELEMETRY");
        telemetry.addData("path state ", pathState);
        telemetry.addData("runShooter? ", runShooter);
        telemetry.addData("runTurret? ", runTurret);
        telemetry.addData("cycles ", numCycles);
        telemetry.addLine("\n");
        telemetry.addLine("PEDRO TELEMETRY");
        telemetry.addData("isBusy? ", follower.isBusy());
        telemetry.addData("x ", follower.getPose().getX());
        telemetry.addData("y ", follower.getPose().getY());
        telemetry.addData("heading ", Math.toDegrees(follower.getPose().getHeading()));
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
                new SleepAction(0.05),
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
                new InstantAction(() -> intake.intake()),
                new SleepAction(1.5),
                new InstantAction(() -> intake.idle())
        ));
    }
}
