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
//import org.firstinspires.ftc.teamcode.Util.RobotHardware;
//import org.firstinspires.ftc.teamcode.Subsystem.LimelightVision;
//
//import org.firstinspires.ftc.teamcode.Subsystem.Intake;
//import org.firstinspires.ftc.teamcode.Subsystem.FlywheelShooter;
//
//@Configurable
//@TeleOp(name = "Iolani TeleOp", group = "C")
//public class IolaniTeleOp extends OpMode {
//
//    private MecanumDrive drive = new MecanumDrive();
//    private RobotHardware robotHardware = new RobotHardware();
//    private ElapsedTime elapsedtime;
//    private List<LynxModule> allHubs;
//    private FtcDashboard dash = FtcDashboard.getInstance();
//    private List<Action> runningActions = new ArrayList<>();
//
//    private LimelightVision llVision = new LimelightVision();
//    private Intake intake = new Intake();
//    private FlywheelShooter shooter = new FlywheelShooter();
//    private Turret turret = new Turret();
//
//    private final double MINIMUM_RPM = 2000;
//    private final double PUSHER_DELAY = 0.5;
//
//    final Gamepad currentGamepad1 = new Gamepad();
//    final Gamepad currentGamepad2 = new Gamepad();
//    final Gamepad previousGamepad1 = new Gamepad();
//    final Gamepad previousGamepad2 = new Gamepad();
//
//    private volatile boolean blueAlliance = true;
//
//    @Override
//    public void init() {
//        robotHardware.initialize(this);
//        intake.initialize(this, robotHardware);
//        drive.initialize(this, robotHardware);
//        llVision.initialize(this, robotHardware);
//        shooter.initialize(this, robotHardware, llVision);
//        turret.initialize(this, robotHardware, llVision, true); // does literally nothing
//
//        // loop time stuff
//        elapsedtime = new ElapsedTime();
//        elapsedtime.reset();
//
//        //activate LL poling
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
//        telemetry.addLine("B for RED (either gamepad)");
//        telemetry.addLine("X for BLUE");
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        // TODO: uncomment below once ready for full teleop testing eventually
//        intake.stowBallPusher();
//        shooter.closeLatch();
//        turret.zero();
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
//        // ALLIANCE SWITCH
//        if (gamepad2.b) {
//            blueAlliance = false;
//        } else if (gamepad2.x) {
//            blueAlliance = true;
//        }
//
//        llVision.trackPose(blueAlliance);
//        intake.ioTeleOp();
//        shooter.ioTeleOp();
//
//        // RIGHT BUMPER shoot one ball at a time
//        if (!shooter.shooterLatchOpen && (shooter.getCurrentRPM() > MINIMUM_RPM) && currentGamepad1.right_bumper) {
//            runningActions.add(new SequentialAction(
//                    new InstantAction(() -> shooter.openLatch()),
//                    new InstantAction(() -> intake.deployBallPusher()),
//                    new SleepAction(PUSHER_DELAY),
//                    new InstantAction(() -> intake.stowBallPusher())
//            ));
//        } else if (shooter.shooterLatchOpen && !currentGamepad1.right_bumper) {
//            runningActions.add(new InstantAction(() -> shooter.closeLatch()));
//        }
//
//        // drive as usual, right bumper slowmode
//        drive.operateSimple();
//
//        /// check google slides for controls
//        /// Shooter is dpad left for automatic after gamepad1 dpad up for 4700, down for 4000, and left for 0
//
//        // telemetry
//        telemetry.addLine("B for RED | X for BLUE");
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
//        telemetry.addData("Loop Times", elapsedtime.milliseconds());
//        telemetry.addData("Limelight Dist", llVision.readDistance());
//        elapsedtime.reset();
//    }
//}
