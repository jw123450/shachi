package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Subsystem.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.Subsystem.Turret;
import org.firstinspires.ftc.teamcode.Util.Globals;
import org.firstinspires.ftc.teamcode.Util.PinpointManager;
import org.firstinspires.ftc.teamcode.Util.RGBLights;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystem.LimelightVision;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.FlywheelShooter;

@TeleOp(name = "Loop Time Diagnosis", group = "B")
public class LoopTimeDiagnosis extends OpMode {

    private ElapsedTime elapsedtime;
    private List<LynxModule> allHubs;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private RobotHardware robotHardware = new RobotHardware();
    private Intake intake = new Intake();
    private Turret turret = new Turret();
    private MecanumDrive drive = new MecanumDrive();
    private LimelightVision llVision = new LimelightVision();
    private FlywheelShooter shooter = new FlywheelShooter();
    private PinpointManager pinpoint = new PinpointManager();
    private RGBLights lights = new RGBLights();

    private final double MINIMUM_RPM = 2000;
    private final double PUSHER_DELAY = 0.5;
    private final double RAPID_FIRE_DELAY = 0.63;
    private final double RGB_ALERT_DELAY = 1.5; // seconds

    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    private volatile boolean blueAlliance = true;
    private volatile boolean useManualIntake = true;
    private volatile boolean vinWantsToShoot = false;
    private volatile boolean cyclingFarZone = false;
    private volatile boolean singleShot = false;
    private int index = 0;

    @Override
    public void init() {
        robotHardware.initialize(this);
        pinpoint.initialize(this, robotHardware);
        intake.initialize(this, robotHardware);
        drive.initialize(this, robotHardware);
        llVision.initialize(this, robotHardware);
        shooter.initialize(this, robotHardware, llVision);
        turret.initialize(this, robotHardware, llVision, false);
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
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
        telemetry.update();
    }

    @Override
    public void start() {
        intake.stowBallPusher();
        shooter.closeLatch();
        pinpoint.transferAutoPose(Globals.autoEndPose);
        shooter.autoRPMmode = true;
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) { hub.clearBulkCache();}
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        // for RR Action execution
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) { if (action.run(packet)) { newActions.add(action); } }
        runningActions = newActions;

        /// ALLIANCE SWITCH
        if (gamepad2.b) {
            blueAlliance = false;
            alertAction(RGBLights.Colors.RED);
        } else if (gamepad2.x) {
            blueAlliance = true;
            alertAction(RGBLights.Colors.BLUE);
        }

        /// CHANGE INDEX ONE AT A TIME
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            index++;
        } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            index--;
        }

        if (index >= 0) {
            // operate loops
            pinpoint.operateTrackingPose(); // Reports X and Y in pedro coordinates
        }
        if (index >= 1) {
            drive.operateSimple();
        }

        if (index >= 2) {
            /// toggling idle RPM
            if (currentGamepad1.x && !previousGamepad1.x) {
                if (cyclingFarZone) { // toggle from far to near
                    cyclingFarZone = false;
                    alertAction(RGBLights.Colors.ORANGE);
                } else { // toggle from near to far
                    cyclingFarZone = true;
                    alertAction(RGBLights.Colors.VIOLET);
                }
            }
        }

        if (index >= 3) {
            /// RAPID FIRE
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && !shooter.shooterLatchOpen) {
                if (turret.targetInRange) {
                    vinWantsToShoot = true;
                } else {
                    alertAction(RGBLights.Colors.YELLOW);
                }
            }
        }

        /// shooter and turret operate loops
        if (index >= 4) {
            shooter.operateOdomTracking(pinpoint.X, pinpoint.Y, blueAlliance, vinWantsToShoot, cyclingFarZone);
        }
        if (index >= 5) {
            turret.operateOdomTracking(pinpoint.X, pinpoint.Y, pinpoint.normalizedHeading, blueAlliance, vinWantsToShoot);
        }

        /// RAPID FIRE
        if (index >= 6) {
            if (vinWantsToShoot && shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
                useManualIntake = false;
                runningActions.add(new SequentialAction(
                        new SleepAction(0.03),
                        new InstantAction(() -> shooter.openLatch()),
                        new InstantAction(() -> intake.intakeFullPower()),
                        new SleepAction(RAPID_FIRE_DELAY),
                        new InstantAction(() -> intake.deployBallPusher()),
                        new SleepAction(PUSHER_DELAY),
                        new InstantAction(() -> intake.stowBallPusher()),
                        new InstantAction(() -> shooter.closeLatch()),
                        new InstantAction(() -> intake.idle()),
                        new InstantAction(() -> useManualIntake = true),
                        new InstantAction(() -> vinWantsToShoot = false)
                ));
            }
        }

        /// intake logic
        if (index >= 7) {
            if (useManualIntake) {
                if (currentGamepad1.right_trigger > 0.9 && intake.intakeState != Intake.IntakeState.INTAKING_FULL_POWER) {
                    intake.intakeFullPower();
                } else if (currentGamepad1.right_trigger > 0.1 && intake.intakeState != Intake.IntakeState.INTAKING) {
                    intake.intake();
                } else if (currentGamepad1.a && !previousGamepad1.a && intake.intakeState != Intake.IntakeState.REVERSE) {
                    intake.reverse();
                } else if (!currentGamepad1.a && previousGamepad1.a && intake.intakeState == Intake.IntakeState.REVERSE) {
                    intake.idle();
                } else if (currentGamepad1.right_trigger <= 0.1 && intake.intakeState != Intake.IntakeState.IDLE && intake.intakeState != Intake.IntakeState.REVERSE) {
                    intake.idle();
                }
            }
            intake.operateCurrentLimiting();
            if (intake.motorStalled) {
                alertAction(RGBLights.Colors.YELLOW);
            }
        }

        // manual odom reset
        if (index >= 8) {
            if (currentGamepad1.y && !previousGamepad1.y) {
                pinpoint.teleOpManualReset(blueAlliance);
                alertAction(RGBLights.Colors.GREEN);
            }
        }

        // telemetry
        telemetry.addLine("\nOTHER");
        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        telemetry.addData("current index", index);
        elapsedtime.reset();
    }

    private void alertAction(RGBLights.Colors alertColor) {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> lights.setColor(alertColor)),
                new SleepAction(RGB_ALERT_DELAY),
                new InstantAction(() -> lights.setColor(RGBLights.Colors.WHITE))
        ));
    }
}
