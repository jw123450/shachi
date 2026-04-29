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

@Autonomous(name = "Far Park Preload ONLY", group = "C", preselectTeleOp = "Full Teleop DUAL DRIVER")
public class FarParkPreloadOnly extends OpMode {

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
    private boolean secondTimeComplete = false;
    private boolean blueAlliance = true;
    private volatile boolean runShooter = false;
    private volatile boolean runTurret = false;
    private volatile boolean currentlyShooting = false;
    private boolean cyclingFarZone = true; // for shooter

    ///  CONSTANTS
    private final double TRANSFER_ONLY_DELAY = 0.03;
    private final double RAPID_FIRE_DELAY = 0.6; // seconds (sleepAction)
    private final double DELAY_BEFORE_MOVING = 150; // milliseconds

    /// BLUE SIDE
    /// blue start 56.4 9 180
    /// red start 90.6 9.8 0
    private final Pose startPoseBlue    = new Pose(56.4,9, Math.toRadians(180));
    private final Pose scorePoseBlue    = new Pose(50.5, 11.5, Math.toRadians(180));
    private final Pose parkPoseBlue     = new Pose(49, 13, Math.toRadians(135));

    /// RED SIDE
    private final Pose startPoseRed = new Pose(90.6, 9.8, Math.toRadians(0));
    private final Pose scorePoseRed = scorePoseBlue.mirror();
    private final Pose parkPoseRed  = parkPoseBlue.mirror();

    // path chains
    private PathChain BScore123, BPark;
    private PathChain RScore123, RPark;

    public void buildPaths() {
        /// BLUE SIDE
        BScore123 = follower.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, scorePoseBlue))
                .setLinearHeadingInterpolation(startPoseBlue.getHeading(), scorePoseBlue.getHeading())
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
                openLatchAction();
                follower.followPath(blueAlliance ? BScore123 : RScore123, 0.5, true);
                setPathState(1);
                break;
            /// SCORE PRELOAD (123)
            case 1:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(2);
                }
                break;
            /// PARK
            case 2:
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
        shooter.initHood();
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
        follower.update();

        telemetry.addLine("B for RED | X for BLUE");
        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");

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
//                new InstantAction(() -> shooter.openLatch()),
//                new SleepAction(LATCH_OPENING_DELAY),
                new InstantAction(() -> intake.runTransferOnly()),
                new SleepAction(TRANSFER_ONLY_DELAY),
                new InstantAction(() -> intake.shootingIntake(cyclingFarZone)),
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
        runningActions.add(new SequentialAction(
                new SleepAction(0.3),
                new InstantAction(() -> shooter.openLatch())
        ));
    }
    private void closeLatchAction() {
        runningActions.add(new InstantAction(() -> shooter.closeLatch()));
    }
}
