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

@Autonomous(name = "21 ball", group = "A", preselectTeleOp = "Full TeleOp")
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
    private int numGateCycles = 0;
    private final int TOTAL_GATE_CYCLES = 3;
    private volatile boolean runShooter = false;
    private volatile boolean runTurret = false;
    private volatile boolean cyclingFarZone = false;
    private volatile boolean currentlyShooting = false;
    private final double TRANSFER_ONLY_DELAY = 0.03;
    private final double RAPID_FIRE_DELAY = 0.4;
    private final double WAIT_GATE_DELAY = 1.5; // TODO

    private final double DELAY_BEFORE_MOVING = 50; // milliseconds
    private final double SLOW_PATH_MAX_POWER = 0.65;
    private final double SLOW_BRAKE_STRENGTH = 1.2;
    private final double AT_POSE_X_TOL = 2.5;
    private final double AT_POSE_Y_TOL = 2.5;


    /// BLUE SIDE POSES
    private final Pose startPoseBlue    = new Pose(31,136.8, Math.toRadians(270));
    private final Pose score123PoseBlue = new Pose(27, 106, Math.toRadians(270));
    private final Pose grab456PoseBlue  = new Pose(21, 91.8, Math.toRadians(270));
    private final Pose score456PoseBlue = new Pose(23.2, 108.7, Math.toRadians(270));
    private final Pose grab789PoseBlue  = new Pose(22.7,64.5, Math.toRadians(270));
    private final Pose scoreGatePoseBlue= new Pose(60.5, 73.5, Math.toRadians(170));
    private final Pose grabGatePoseBlue = new Pose(15, 62.45, Math.toRadians(152));
//    private final Pose grabGateControlPointBlue = new Pose(0, 0, 0);
    private final Pose parkPoseBlue     = new Pose(58, 72, Math.toRadians(225));

    /// RED SIDE POSES
    private final Pose startPoseRed     = startPoseBlue.mirror();
    private final Pose score123PoseRed  = score123PoseBlue.mirror();
    private final Pose grab456PoseRed   = grab456PoseBlue.mirror();
    private final Pose score456PoseRed  = score456PoseBlue.mirror();
    private final Pose grab789PoseRed   = grab789PoseBlue.mirror();
///                                       turnTo 190 deg here
    private final Pose scoreGatePoseRed = scoreGatePoseBlue.mirror();
    private final Pose grabGatePoseRed  = grabGatePoseBlue.mirror();
    private final Pose parkPoseRed      = parkPoseBlue.mirror();

    // PathChains
    private PathChain BScore123, BGrab456, BScore456, BGrab789, BScore789;
    private PathChain BGrabGate, BScoreGate, BPark;
    private PathChain RScore123, RGrab456, RScore456, RGrab789, RScore789;
    private PathChain RGrabGate, RScoreGate, RPark;

    public void buildPaths() {
        /// BLUE SIDE
        BScore123 = follower.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, score123PoseBlue))
                    .setLinearHeadingInterpolation(startPoseBlue.getHeading(), score123PoseBlue.getHeading())
//                .setGlobalDeceleration()
                .build();

        BGrab456 = follower.pathBuilder()
                .addPath(new BezierLine(score123PoseBlue, grab456PoseBlue))
                .setLinearHeadingInterpolation(score123PoseBlue.getHeading(), grab456PoseBlue.getHeading())
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
                .addPath(new BezierLine(grab789PoseBlue, scoreGatePoseBlue))
                .setLinearHeadingInterpolation(scoreGatePoseBlue.getHeading(), scoreGatePoseBlue.getHeading())
                .build();

        BGrabGate = follower.pathBuilder()
                .addPath(new BezierLine(scoreGatePoseBlue, grabGatePoseBlue))
                .setLinearHeadingInterpolation(scoreGatePoseBlue.getHeading(), grabGatePoseBlue.getHeading())
///                .setGlobalDeceleration(1)
                .build();

        BScoreGate = follower.pathBuilder()
                .addPath(new BezierLine(grabGatePoseBlue, scoreGatePoseBlue))
                .setLinearHeadingInterpolation(grabGatePoseBlue.getHeading(), scoreGatePoseBlue.getHeading())
                .build();

        BPark = follower.pathBuilder()
                .addPath(new BezierLine(scoreGatePoseBlue, parkPoseBlue))
                .setLinearHeadingInterpolation(scoreGatePoseBlue.getHeading(), parkPoseBlue.getHeading())
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
                .addPath(new BezierLine(score123PoseRed, grab456PoseRed))
                .setLinearHeadingInterpolation(score123PoseRed.getHeading(), grab456PoseRed.getHeading())
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
                .addPath(new BezierLine(grab789PoseRed, scoreGatePoseRed))
                .setLinearHeadingInterpolation(scoreGatePoseRed.getHeading(), scoreGatePoseRed.getHeading())
                .build();

        RGrabGate = follower.pathBuilder()
                .addPath(new BezierLine(scoreGatePoseRed, grabGatePoseRed))
                .setLinearHeadingInterpolation(scoreGatePoseRed.getHeading(), grabGatePoseRed.getHeading())
///                .setGlobalDeceleration(1)
                .build();

        RScoreGate = follower.pathBuilder()
                .addPath(new BezierLine(grabGatePoseRed, scoreGatePoseRed))
                .setLinearHeadingInterpolation(grabGatePoseRed.getHeading(), scoreGatePoseRed.getHeading())
                .build();

        RPark = follower.pathBuilder()
                .addPath(new BezierLine(scoreGatePoseRed, parkPoseRed))
                .setLinearHeadingInterpolation(scoreGatePoseRed.getHeading(), parkPoseRed.getHeading())
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
                follower.followPath(blueAlliance ? BScore123 : RScore123);
                setPathState(1);
                break;
            /// SCORE PRELOAD
            case 1:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(2);
                }
                break;
            case 2:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
//                    runShooter = true;
                    shooter.closeLatch();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrab456 : RGrab456, 0.8, true);
                    setPathState(3);
                }
                break;
            /// GRAB S1 (456)
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
            /// SCORE S1 (456)
            case 4:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(5);
                }
                break;
            case 5:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
//                    runShooter = false;
                    intake.intakingIntake();
                    shooter.closeLatch();
                    follower.followPath(blueAlliance ? BGrab789 : RGrab789, true);
                    setPathState(7);
                }
                break;
            /// GRAB S2 (789)
//            case 6: // TODO test diff condition
//                if (!follower.isBusy()) {
//                    delayedIdleAction();
////                    follower.turnTo(blueAlliance ? scoreGatePoseBlue.getHeading() : scoreGatePoseRed.getHeading());
//                    setPathState(7);
//                }
//                break;
            // TURN IN PLACE
            case 7:
                if (!follower.isBusy()) {
//                    delayedIdleAction();
                    intake.idle();
                    shooter.openLatch();
                    runShooter = true;
                    follower.followPath(blueAlliance ? BScore789 : RScore789, true);
                    setPathState(8);
                }
                break;
            /// SCORE S2 (789)
            case 8:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(9);
                }
                break;
            /// GRAB GATE
            case 9:
                if (!currentlyShooting && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    intake.deployIntake();
                    intake.intakingIntake();
                    follower.followPath(blueAlliance ? BGrabGate : RGrabGate, true);
                    setPathState(10);
                }
                break;
            // OPEN GATE
            case 10:
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;
            // WAIT UNTIL INTAKE FULL OR TIME LIMIT PASSED
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > WAIT_GATE_DELAY || intake.isFull) {
                    intake.idle();
                    intake.stowIntake();
                    shooter.openLatch();
                    runShooter = true;
                    follower.followPath(blueAlliance ? BScoreGate : RScoreGate, true);
                    setPathState(12);
                }
                break;
            /// SCORE GATE
            case 12:
                if (!follower.isBusy() && shooter.atTargetRPM && turret.atTargetAngle) {
                    currentlyShooting = true;
                    rapidFireAction();
                    setPathState(13);
                }
                break;
            /// LOOP BACK OR PARK
            case 13:
                if (!shooter.latchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
                    runShooter = false;
                    numGateCycles++;
                    if (numGateCycles <= TOTAL_GATE_CYCLES) {
                        // run it back
                        intake.deployIntake();
                        intake.intakingIntake();
                        follower.followPath(blueAlliance ? BGrabGate : RGrabGate, true);
                        setPathState(10);
                    } else {
                        // park
                        follower.followPath(blueAlliance ? BPark : RPark, true);
                        setPathState(-1);
                    }
                }
                break;
            /// SHUT DOWN
            case -1:
                if (!shooter.latchOpen && pathTimer.getElapsedTime() > DELAY_BEFORE_MOVING) {
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

        intake.operateAuto();
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
