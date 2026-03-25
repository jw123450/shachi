package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Turret;
import org.firstinspires.ftc.teamcode.Util.Globals;
import org.firstinspires.ftc.teamcode.Util.PinpointManager;
import org.firstinspires.ftc.teamcode.Util.RGBLights;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Full Teleop", group = "A")
public class FullTeleOp extends OpMode {

    private ElapsedTime elapsedtime;
    private List<LynxModule> allHubs;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private RobotHardware robotHardware = new RobotHardware();
    private Intake intake = new Intake();
    private Turret turret = new Turret();
    private MecanumDrive drive = new MecanumDrive();
//    private LimelightVision llVision = new LimelightVision();
    private Shooter shooter = new Shooter();
    private PinpointManager pinpoint = new PinpointManager();
    private RGBLights lights = new RGBLights();

    private final double MINIMUM_RPM = 2000;
    private final double LATCH_OPENING_DELAY = 0.45;
    private final double TRANSFER_ONLY_DELAY = 0.03;
    private final double SINGLE_SHOT_DELAY = 0.3;
    private final double RAPID_FIRE_DELAY = 0.63;
    private final double RGB_ALERT_DELAY = 1.5; // seconds

    private volatile boolean blueAlliance = true;
    private volatile boolean useManualIntake = true;
    private volatile boolean vinWantsToShoot = false;
    private volatile boolean cyclingFarZone = false;
    private volatile boolean singleShot = false;

    @Override
    public void init() {
        robotHardware.initialize(this);
        pinpoint.initialize(this, robotHardware);
        intake.initialize(this, robotHardware);
        drive.initialize(this, robotHardware);
//        llVision.initialize(this, robotHardware);
        shooter.initialize(this, robotHardware);
        turret.initialize(this, robotHardware);
        lights.initialize(this, robotHardware);

        // loop time stuff
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        // Pull info from globals
        pinpoint.transferAutoPose(Globals.autoEndPose);
        blueAlliance = Globals.blueAlliance;
        lights.setColor(blueAlliance ? RGBLights.Colors.BLUE : RGBLights.Colors.RED);

        // bulk caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
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

        pinpoint.operateTrackingPose();

        telemetry.addLine("B for RED (either gamepad)");
        telemetry.addLine("X for BLUE");
        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");

        telemetry.addData("end pose X", Globals.autoEndPose.getX());
        telemetry.addData("end pose Y", Globals.autoEndPose.getY());
        telemetry.addData("end pose heading", Math.toDegrees(Globals.autoEndPose.getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        turret.setBoth(0.5);
        shooter.closeLatch();
        intake.stowIntake();
        shooter.initHood();
        pinpoint.transferAutoPose(Globals.autoEndPose);
        shooter.autoRPMmode = true;
        lights.setColor(RGBLights.Colors.WHITE);
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) { hub.clearBulkCache();}
        // for RR Action execution
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) { if (action.run(packet)) { newActions.add(action); } }
        runningActions = newActions;

        // operate loops
        pinpoint.operateTrackingPose(); // Changes X and Y to pedro coordinates
        drive.operateSimple();

        /// REQUEST RAPID FIRE
        if (gamepad2.leftBumperWasPressed() && useManualIntake) { // useManualIntake means not in middle of shot
            // get ready to shoot
            vinWantsToShoot = true;
            singleShot = false;
            shooter.openLatch();
        }
        /// REQUEST SINGLE SHOT
        else if (gamepad2.dpadLeftWasPressed() && useManualIntake) {
            vinWantsToShoot = true;
            singleShot = true;
            shooter.openLatch();
        }

        if (gamepad2.dpadDownWasPressed() && vinWantsToShoot && useManualIntake) { // cancels shot if bugging
            vinWantsToShoot = false;
            singleShot = false;
            shooter.closeLatch();
        }

        // more operate loops (ugly to put here, but fixes small bug)
        turret.operateBasic(pinpoint.X, pinpoint.Y, pinpoint.normalizedHeading, blueAlliance, vinWantsToShoot);
        shooter.operateBasic(pinpoint.X, pinpoint.Y, blueAlliance, vinWantsToShoot, cyclingFarZone);
//        shootWhileMoveCalcsSimple();

        // SHOOTER
        if (vinWantsToShoot && (!gamepad2.left_bumper || singleShot)) {
            /// below condition is where robot sometimes get stuck trying but failing to shoot
            if (shooter.atTargetRPM && turret.atTargetAngle && shooter.latchOpen) {
                if (singleShot) {
                    /// SINGLE SHOT
                    useManualIntake = false;
                    runningActions.add(new SequentialAction(
//                            new InstantAction(() -> shooter.openLatch()),
//                            new SleepAction(LATCH_OPENING_DELAY),
                            new InstantAction(() -> intake.runTransferOnly()),
                            new SleepAction(SINGLE_SHOT_DELAY),
                            new InstantAction(() -> shooter.closeLatch()),
                            new InstantAction(() -> intake.idle()),
                            new InstantAction(() -> useManualIntake = true),
                            new InstantAction(() -> singleShot = false),
                            new InstantAction(() -> vinWantsToShoot = false)
                    ));
                } else {
                    /// RAPID FIRE
                    useManualIntake = false;
                    runningActions.add(new SequentialAction(
//                            new InstantAction(() -> shooter.openLatch()),
//                            new SleepAction(LATCH_OPENING_DELAY),
                            new InstantAction(() -> intake.runTransferOnly()),
                            new SleepAction(TRANSFER_ONLY_DELAY),
                            new InstantAction(() -> intake.shootingIntake()),
                            new SleepAction(RAPID_FIRE_DELAY),
                            new InstantAction(() -> shooter.closeLatch()),
                            new InstantAction(() -> intake.idle()),
                            new InstantAction(() -> useManualIntake = true),
                            new InstantAction(() -> vinWantsToShoot = false)
                    ));
                }
            }
        }

        intake.operateTeleOp(useManualIntake);

        // Reset functions
//        if (gamepad1.left_trigger > 0.8) {
//            // limelight pose reset
//            llVision.trackPose(blueAlliance);
//            if (llVision.tagSeen) {
//                Pose currentLLPose = llVision.absRelocalize(Math.toRadians(pinpoint.normalizedHeading));
//                if (gamepad1.yWasPressed()) {
//                    if (currentLLPose.getX() == 0 || currentLLPose.getY() == 0) {
//                        alertAction(RGBLights.Colors.YELLOW);
//                    } else {
//                        pinpoint.teleOpAprilTagReset(currentLLPose);
//                        alertAction(RGBLights.Colors.GREEN);
//                    }
//                }
//                telemetry.addData("LL X", currentLLPose.getX());
//                telemetry.addData("LL Y", currentLLPose.getY());
//            }
//
//            // manual pinpoint heading reset
//            if (gamepad1.bWasPressed()) {
//                pinpoint.teleOpResetHeading();
//                alertAction(RGBLights.Colors.GREEN);
//            }
//        }

        // toggling idle RPM
        if (gamepad1.xWasPressed()) {
            if (cyclingFarZone) { // toggle from far to near
                cyclingFarZone = false;
                alertAction(RGBLights.Colors.YELLOW);
            } else { // toggle from near to far
                cyclingFarZone = true;
                alertAction(RGBLights.Colors.VIOLET);
            }
        }

        // default to orange or violet when no active alerts
        if (lights.currentColor == RGBLights.Colors.WHITE) {
            runningActions.add(new InstantAction(() -> lights.setColor(cyclingFarZone ? RGBLights.Colors.VIOLET : RGBLights.Colors.YELLOW)));
        }

        // ALLIANCE SWITCH
        if (gamepad2.b) {
            blueAlliance = false;
            alertAction(RGBLights.Colors.RED);
        } else if (gamepad2.x) {
            blueAlliance = true;
            alertAction(RGBLights.Colors.BLUE);
        }

        // manual driver 2 goal pose adjust
        if (gamepad2.right_bumper) {
            if (gamepad2.dpadUpWasPressed()) {
                adjustGoalPose(false, 1);
                microAlert(RGBLights.Colors.GREEN);
            } else if (gamepad2.dpadDownWasPressed()) {
                adjustGoalPose(false, -1);
                microAlert(RGBLights.Colors.GREEN);
            } else if (gamepad2.dpadRightWasPressed()) {
                adjustGoalPose(true, 1);
                microAlert(RGBLights.Colors.BLUE);
            } else if (gamepad2.dpadLeftWasPressed()) {
                adjustGoalPose(true, -1);
                microAlert(RGBLights.Colors.BLUE);
            } else if (gamepad2.leftBumperWasPressed()) {
                // full reset
                Globals.blueGoalX = 10.5;
                Globals.blueGoalY = 139;
                Globals.redGoalX = 134.5;
                Globals.redGoalY = 139;
            }
        }

        // telemetry
        telemetry.addLine("\nOTHER");
        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
        telemetry.addData("cyclingFarZone", cyclingFarZone);
        telemetry.addData("Loop Times", elapsedtime.milliseconds());

//        if (blueAlliance) {
//            telemetry.addData("Blue Goal X", Globals.blueGoalX);
//            telemetry.addData("Blue Goal Y", Globals.blueGoalY);
//        } else {
//            telemetry.addData("Red Goal X", Globals.redGoalX);
//            telemetry.addData("Red Goal Y", Globals.redGoalY);
//        }
//        telemetry.addData("LL X", currentLLPose.getX());
//        telemetry.addData("LL Y", currentLLPose.getY());
        telemetry.addLine("\nPOSE");
        telemetry.addData("pp X", pinpoint.X);
        telemetry.addData("pp Y", pinpoint.Y);
        telemetry.addData("pp heading (deg)", pinpoint.normalizedHeading);
        elapsedtime.reset();
    }

    @Override
    public void stop() {
        Globals.blueGoalX = 10.5;
        Globals.blueGoalY = 139;
        Globals.redGoalX = 134.5;
        Globals.redGoalY = 139;
    }

    private void alertAction(RGBLights.Colors alertColor) {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> lights.setColor(alertColor)),
                new SleepAction(RGB_ALERT_DELAY),
                new InstantAction(() -> lights.setColor(RGBLights.Colors.WHITE))
        ));
    }

    private void microAlert(RGBLights.Colors alertColor) {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> lights.setColor(alertColor)),
                new SleepAction(0.2),
                new InstantAction(() -> lights.setColor(RGBLights.Colors.WHITE))
        ));
    }

    private void adjustGoalPose(boolean changeX, int dir) {
        if (changeX) {
            if (blueAlliance) {
                Globals.blueGoalX += dir;
            } else {
                Globals.redGoalX += dir;
            }
        } else {
            if (blueAlliance) {
                Globals.blueGoalY += dir;
            } else {
                Globals.redGoalY += dir;
            }
        }
    }
}
