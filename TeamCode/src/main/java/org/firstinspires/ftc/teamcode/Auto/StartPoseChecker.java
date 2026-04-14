package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

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

@Autonomous(name = "start pose checker", group = "C")
public class StartPoseChecker extends OpMode {
    /// I promise you don't wanna look at this code
    /// Your eyes might melt if you see how deep fried this is
    /// Just close the file and trust that it works

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private List<LynxModule> allHubs;
    private ElapsedTime elapsedtime;

    private RobotHardware robotHardware = new RobotHardware();
    private RGBLights lights = new RGBLights();

    ///  CONSTANTS
    private boolean blueAlliance = true;

    /// /////////  /////////  /////////  /////////  /////////  /////////  /////////  /////////  /////////
    private volatile boolean teleopTakeover = false;
    private int num = 0;
    /// /////////  /////////  /////////  /////////  /////////  /////////  /////////  /////////  /////////

    /// BLUE SIDE POSES
    private final Pose startPoseBlue18 = new Pose(31,136.3, Math.toRadians(270));
    private final Pose startPoseBlue21 = new Pose(20,115, Math.toRadians(143));
    private final Pose startPoseBlueFar = new Pose(54.3,8.3, Math.toRadians(180));
    private final Pose fieldMiddle = new Pose(72, 72, Math.toRadians(270));

    /// RED SIDE POSES
    private final Pose startPoseRed18 = startPoseBlue18.mirror();
    private final Pose startPoseRed21 = startPoseBlue21.mirror();
    private final Pose startPoseRedFar = startPoseBlueFar.mirror();

    // PathChains
    private PathChain B18ToMiddle, B21ToMiddle, BFarToMiddle, R18ToMiddle, R21ToMiddle, RFarToMiddle;

    public void buildPaths() {
        B18ToMiddle = follower.pathBuilder()
                .addPath(new BezierLine(startPoseBlue18, fieldMiddle))
                .setLinearHeadingInterpolation(startPoseBlue18.getHeading(), startPoseBlue18.getHeading())
                .build();

        B21ToMiddle = follower.pathBuilder()
                .addPath(new BezierLine(startPoseBlue21, fieldMiddle))
                .setLinearHeadingInterpolation(startPoseBlue21.getHeading(), startPoseBlue21.getHeading())
                .build();

        BFarToMiddle = follower.pathBuilder()
                .addPath(new BezierLine(startPoseBlue18, fieldMiddle))
                .setLinearHeadingInterpolation(startPoseBlueFar.getHeading(), startPoseBlueFar.getHeading())
                .build();

        R18ToMiddle = follower.pathBuilder()
                .addPath(new BezierLine(startPoseRed18, fieldMiddle))
                .setLinearHeadingInterpolation(startPoseBlue18.getHeading(), startPoseRed18.getHeading())
                .build();

        R21ToMiddle = follower.pathBuilder()
                .addPath(new BezierLine(startPoseRed21, fieldMiddle))
                .setLinearHeadingInterpolation(startPoseBlue21.getHeading(), startPoseRed21.getHeading())
                .build();

        RFarToMiddle = follower.pathBuilder()
                .addPath(new BezierLine(startPoseRedFar, fieldMiddle))
                .setLinearHeadingInterpolation(startPoseBlueFar.getHeading(), startPoseRedFar.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                switch (num) {
                    case 0:
                        follower.followPath(blueAlliance ? B18ToMiddle : R18ToMiddle);
                        break;
                    case 1:
                        follower.followPath(blueAlliance ? B21ToMiddle : R21ToMiddle);
                        break;
                    case 2:
                        follower.followPath(blueAlliance ? BFarToMiddle : RFarToMiddle);
                        break;
                    case 3:
                        follower.holdPoint(fieldMiddle);
                        break;
                    default:
                        follower.holdPoint(fieldMiddle);
                        break;
                }
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.holdPoint(fieldMiddle);
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        lights.initialize(this, robotHardware);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }
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

        if (gamepad1.dpadDownWasPressed()) {
            num--;
        } else if (gamepad1.dpadUpWasPressed()) {
            num++;
        }

        follower.update();

        telemetry.addLine("B for RED | X for BLUE");
        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
        telemetry.addLine("0 -> 18 ball");
        telemetry.addLine("1 -> 21 ball");
        telemetry.addLine("2 -> far auto");
        telemetry.addLine("3 -> 18 but smart method");
        telemetry.addLine("out of range -> 18 but smart method");
        telemetry.addData("num", num);
//        telemetry.addData("x ", follower.getPose().getX());
//        telemetry.addData("y ", follower.getPose().getY());
//        telemetry.addData("heading ", Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.addData("velocity magnitude", Math.abs(follower.getVelocity().getMagnitude()));
//        telemetry.addData("angular velocity", Math.abs(follower.getAngularVelocity()));
//        if (Math.abs(follower.getVelocity().getMagnitude()) > 1 || Math.abs(follower.getAngularVelocity()) > 0.2) {
//            telemetry.addLine("PINPOINT IS COOKED");
//            telemetry.addLine("PINPOINT IS COOKED");
//            telemetry.addLine("PINPOINT IS COOKED");
//        }
        telemetry.update();
    }

    @Override
    public void start() {
        switch (num) {
            case 0:
                follower.setStartingPose(blueAlliance ? startPoseBlue18 : startPoseRed18);
                break;
            case 1:
                follower.setStartingPose(blueAlliance ? startPoseBlue21 : startPoseRed21);
                break;
            case 2:
                follower.setStartingPose(blueAlliance ? startPoseBlueFar : startPoseRedFar);
                break;
            case 3:
                follower.setStartingPose(blueAlliance ? startPoseBlue18 : startPoseRed18);
                break;
            default:
                follower.setStartingPose(blueAlliance ? startPoseBlue18 : startPoseRed18);
                break;
        }

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

        // loops
        if (!teleopTakeover) {
            autonomousPathUpdate();
        } else {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }
        follower.update();

        if (gamepad1.aWasPressed()) {
            teleopTakeover = true;
            follower.startTeleopDrive();
        }

        Pose currentPose = follower.getPose();
        // Feedback to Driver Hub for debugging
        telemetry.addLine("STATE TELEMETRY");
        telemetry.addData("path state ", pathState);
        telemetry.addData("teleopTakeover ", teleopTakeover);
        telemetry.addData("x ", currentPose.getX());
        telemetry.addData("y ", currentPose.getY());
        telemetry.addData("heading ", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("isBusy? ", follower.isBusy());
        telemetry.addData("isRobotStuck? ", follower.isRobotStuck());
        telemetry.addLine("\n");
        telemetry.addData("looptimes ", elapsedtime.milliseconds());

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
}
