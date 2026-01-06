//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.bylazar.configurables.annotations.Configurable;
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
//import org.firstinspires.ftc.teamcode.Util.RGBLights;
//import org.firstinspires.ftc.teamcode.Util.RobotHardware;
//import org.firstinspires.ftc.teamcode.Subsystem.LimelightVision;
//
//import org.firstinspires.ftc.teamcode.Subsystem.Intake;
//import org.firstinspires.ftc.teamcode.Subsystem.FlywheelShooter;
//
//@Configurable
//@TeleOp(name = "Kauai TeleOp", group = "A")
//public class KauaiTeleOp extends OpMode {
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
//    private RGBLights lights = new RGBLights();
//
//    private final double MINIMUM_RPM = 2000;
//    private final double PUSHER_DELAY = 0.5;
//    private final double RAPID_FIRE_DELAY = 0.6;
//    private final double RGB_ALERT_DELAY = 2; // seconds
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
//        intake.initialize(this, robotHardware);
//        drive.initialize(this, robotHardware);
//        llVision.initialize(this, robotHardware);
//        shooter.initialize(this, robotHardware, llVision);
//        turret.initialize(this, robotHardware, llVision, false);
//        lights.initialize(this, robotHardware);
//
//        // loop time stuff
//        elapsedtime = new ElapsedTime();
//        elapsedtime.reset();
//
//        // pull info from globals
//        blueAlliance = Globals.blueAlliance;
//        lights.setColor(blueAlliance ? RGBLights.Colors.BLUE : RGBLights.Colors.RED);
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
//            lights.setColor(RGBLights.Colors.RED);
//        } else if (gamepad1.x || gamepad2.x) {
//            blueAlliance = true;
//            lights.setColor(RGBLights.Colors.BLUE);
//        }
//
//        telemetry.addLine("B for RED (either gamepad)");
//        telemetry.addLine("X for BLUE");
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
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
//            alertAction(RGBLights.Colors.RED);
//        } else if (gamepad2.x) {
//            blueAlliance = true;
//            alertAction(RGBLights.Colors.BLUE);
//        }
//
//        // Turret set pose logic
//        if (currentGamepad1.x &&!previousGamepad1.x) { // use set turret poses for aiming from far zone
//            turret.useLL = false;
//            turret.toggleSetPoses(blueAlliance); // 69° and 110° set pose controls for turret
//            alertAction(RGBLights.Colors.VIOLET);
//        } else if (currentGamepad1.y && !previousGamepad1.y) { // back to using LL
//            turret.useLL = true;
//            alertAction(RGBLights.Colors.GREEN);
//        }
//
//        // Loops
//        llVision.trackPose(blueAlliance);
//        shooter.kauaiTeleOp();
//        turret.kauaiTeleOp(blueAlliance);
//        drive.operateSimple();
//
//        //  RAPID FIRE
//        if (!shooter.shooterLatchOpen && (shooter.autoRPMmode ? llVision.tagDetected(blueAlliance) : true) && currentGamepad1.right_bumper && !previousGamepad1.right_bumper && shooter.atTargetRPM) { // rising edge
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
//        // warning "strobe" to let vincent know his shooter is not spinning
//        if (shooter.getCurrentRPM() < 100) {
//            if (lights.currentColor != (blueAlliance ? RGBLights.Colors.BLUE : RGBLights.Colors.RED)) {
//                lights.setColor(blueAlliance ? RGBLights.Colors.BLUE : RGBLights.Colors.RED);
//            } else if (lights.currentColor != RGBLights.Colors.WHITE) {
//                lights.setColor(RGBLights.Colors.WHITE);
//            }
//        }
//
//        // telemetry
//        telemetry.addLine("B for RED | X for BLUE");
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
//        telemetry.addData("Loop Times", elapsedtime.milliseconds());
//        telemetry.addData("Limelight Dist", llVision.readDistance());
//        elapsedtime.reset();
//    }
//
//    private void alertAction(RGBLights.Colors alertColor) {
//        runningActions.add(new SequentialAction(
//                new InstantAction(() -> lights.setColor(alertColor)),
//                new SleepAction(RGB_ALERT_DELAY),
//                new InstantAction(() -> lights.setColor(RGBLights.Colors.WHITE))
//        ));
//    }
//}
