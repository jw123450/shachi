//package org.firstinspires.ftc.teamcode.TeleOp.Testing;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import org.firstinspires.ftc.teamcode.Subsystem.MecanumDrive;
//
//import java.util.ArrayList;
//import java.util.List;
//
//import org.firstinspires.ftc.teamcode.Subsystem.Turret;
//import org.firstinspires.ftc.teamcode.Util.Globals;
//import org.firstinspires.ftc.teamcode.Util.PinpointManager;
//import org.firstinspires.ftc.teamcode.Util.RobotHardware;
//import org.firstinspires.ftc.teamcode.Subsystem.LimelightVision;
//
//import org.firstinspires.ftc.teamcode.Subsystem.Intake;
//import org.firstinspires.ftc.teamcode.Subsystem.FlywheelShooter;
//
//@TeleOp(name = "Odom Tracking Turret + Shooter", group = "Testing")
//public class OdomTrackingTest extends OpMode {
//
//    private ElapsedTime elapsedtime;
//    private List<LynxModule> allHubs;
//    private FtcDashboard dash = FtcDashboard.getInstance();
//    private List<Action> runningActions = new ArrayList<>();
//
//    private RobotHardware robotHardware = new RobotHardware();
//    private Intake intake = new Intake();
//    private Turret turret = new Turret();
//    private MecanumDrive drive = new MecanumDrive();
//    private LimelightVision llVision = new LimelightVision();
//    private FlywheelShooter shooter = new FlywheelShooter();
//    public PinpointManager pinpoint = new PinpointManager();
//
//    private final double MINIMUM_RPM = 2000;
//    private final double PUSHER_DELAY = 0.5;
//    private final double RAPID_FIRE_DELAY = 0.7;
//
//    final Gamepad currentGamepad1 = new Gamepad();
//    final Gamepad currentGamepad2 = new Gamepad();
//    final Gamepad previousGamepad1 = new Gamepad();
//    final Gamepad previousGamepad2 = new Gamepad();
//
//    private volatile boolean blueAlliance = true;
//    private volatile boolean useManualIntake = true;
//
//    @Override
//    public void init() {
//        robotHardware.initialize(this);
//        pinpoint.initialize(this, robotHardware);
//        intake.initialize(this, robotHardware);
//        drive.initialize(this, robotHardware);
//        llVision.initialize(this, robotHardware);
//        shooter.initialize(this, robotHardware, llVision);
//        turret.initialize(this, robotHardware, llVision, false);
//
//        // loop time stuff
//        elapsedtime = new ElapsedTime();
//        elapsedtime.reset();
//
//        pinpoint.transferAutoPose(Globals.autoEndPose);
//
//        // bulk caching
//        allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
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
//        pinpoint.operateTrackingPose();
//
//        telemetry.addLine("B for RED (either gamepad)");
//        telemetry.addLine("X for BLUE");
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
//
//        telemetry.addData("end pose X", Globals.autoEndPose.getX());
//        telemetry.addData("end pose Y", Globals.autoEndPose.getY());
//        telemetry.addData("end pose heading", Globals.autoEndPose.getHeading());
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        pinpoint.transferAutoPose(Globals.autoEndPose);
//        intake.stowBallPusher();
//        shooter.closeLatch();
//    }
//
//    @Override
//    public void loop() {
//        for (LynxModule hub : allHubs) { hub.clearBulkCache();}
//        previousGamepad1.copy(currentGamepad1);
//        previousGamepad2.copy(currentGamepad2);
//        currentGamepad1.copy(gamepad1);
//        currentGamepad2.copy(gamepad2);
//        // for RR Action execution
//        TelemetryPacket packet = new TelemetryPacket();
//        List<Action> newActions = new ArrayList<>();
//        for (Action action : runningActions) { if (action.run(packet)) { newActions.add(action); } }
//        runningActions = newActions;
//
//        /// check google slides for controls
//
//        // ALLIANCE SWITCH
//        if (gamepad2.b) {
//            blueAlliance = false;
//        } else if (gamepad2.x) {
//            blueAlliance = true;
//        }
//
//        pinpoint.operateTrackingPose();
//        intake.operateCurrentLimiting();
//        shooter.operateOdomTracking(pinpoint.X, pinpoint.Y, blueAlliance, gamepad1.dpad_left, false);
//        turret.operateOdomTracking(pinpoint.X, pinpoint.Y, pinpoint.normalizedHeading, blueAlliance, gamepad1.dpad_left);
//        drive.operateSimple();
//
//        ///  RAPID FIRE
//        /// press button, and robot shoots all three, just need to stay still
//        if (!shooter.shooterLatchOpen && currentGamepad1.right_bumper && !previousGamepad1.right_bumper && shooter.atTargetRPM) { // rising edge
//            useManualIntake = false;
//            runningActions.add(new SequentialAction(
//                    new InstantAction(() -> shooter.openLatch()),
//                    new InstantAction(() -> intake.intakeFullPower()),
//                    new SleepAction(RAPID_FIRE_DELAY),
//                    new InstantAction(() -> intake.deployBallPusher()),
//                    new SleepAction(PUSHER_DELAY),
//                    new InstantAction(() -> intake.stowBallPusher()),
//                    new InstantAction(() -> shooter.closeLatch()),
//                    new InstantAction(() -> intake.idle()),
//                    new InstantAction(() -> useManualIntake = true)
//            ));
//        }
//
//        // intake logic
//        if (useManualIntake) {
//            if (currentGamepad1.right_trigger > 0.9 && intake.intakeState != Intake.IntakeState.INTAKING_FULL_POWER) {
//                intake.intakeFullPower();
//            } else if (currentGamepad1.right_trigger > 0.1 && intake.intakeState != Intake.IntakeState.INTAKING) {
//                intake.intake();
//            } else if (currentGamepad1.a && !previousGamepad1.a && intake.intakeState != Intake.IntakeState.REVERSE) {
//                intake.reverse();
//            } else if (!currentGamepad1.a && previousGamepad1.a && intake.intakeState == Intake.IntakeState.REVERSE) {
//                intake.idle();
//            } else if (currentGamepad1.right_trigger <= 0.1 && intake.intakeState != Intake.IntakeState.IDLE && intake.intakeState != Intake.IntakeState.REVERSE) {
//                intake.idle();
//            }
//        }
//
//
//        // telemetry
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
//        telemetry.addData("Loop Times", elapsedtime.milliseconds());
////        telemetry.addData("Tag Detected?", llVision.tagDetected(blueAlliance));
//        elapsedtime.reset();
//    }
//}
