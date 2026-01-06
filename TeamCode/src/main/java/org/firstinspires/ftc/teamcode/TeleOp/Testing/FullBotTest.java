//package org.firstinspires.ftc.teamcode.TeleOp.Testing;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import org.firstinspires.ftc.teamcode.Subsystem.MecanumDrive;
//import java.util.List;
//
//import org.firstinspires.ftc.teamcode.Subsystem.turretMotor;
//import org.firstinspires.ftc.teamcode.Util.RobotHardware;
//import org.firstinspires.ftc.teamcode.Subsystem.LimelightVision;
//import com.bylazar.configurables.annotations.IgnoreConfigurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//
//import org.firstinspires.ftc.teamcode.Subsystem.Intake;
//import org.firstinspires.ftc.teamcode.Subsystem.FlywheelShooter;
//import org.firstinspires.ftc.teamcode.Subsystem.turretMotor;
//
//@Configurable
//@TeleOp(name = "Iolani TeleOp", group = "Testing")
//public class FullBotTest extends OpMode {
//
//
//    private MecanumDrive drive = new MecanumDrive(); // TODO: motor + pinpoint config
//    private RobotHardware robotHardware = new RobotHardware();
//    private ElapsedTime elapsedtime;
//    private List<LynxModule> allHubs;
//
//    private LimelightVision llVision = new LimelightVision();
//
//    private Intake intake = new Intake();
//    private FlywheelShooter flywheelShooter = new FlywheelShooter();
//    private turretMotor turret = new turretMotor();
//
//    @IgnoreConfigurable
////    static TelemetryManager telemetryM;
//
//    final Gamepad currentGamepad1 = new Gamepad();
//    final Gamepad currentGamepad2 = new Gamepad();
//    final Gamepad previousGamepad1 = new Gamepad();
//    final Gamepad previousGamepad2 = new Gamepad();
//
//    @Override
//    public void init() {
//        robotHardware.initialize(this);
//        drive.initialize(this, llVision, robotHardware);
//        intake.initialize(this, robotHardware);
//        flywheelShooter.initialize(this, robotHardware);
//        turret.initialize(this, robotHardware); //nothing else with this yet
//
//        // loop time stuff
//        elapsedtime = new ElapsedTime();
//        elapsedtime.reset();
//
//        // panels display
////        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        //activate LL poling
//      //  llVision.initialize(this, robotHardware);
//
//        // bulk caching
//        allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
//
//    }
//
//    @Override
//    public void loop() {
//        for (LynxModule hub : allHubs) {
//            hub.clearBulkCache();
//        }
//
//        //  rising edge detection
//        previousGamepad1.copy(currentGamepad1);
//        previousGamepad2.copy(currentGamepad2);
//        currentGamepad1.copy(gamepad1);
//        currentGamepad2.copy(gamepad2);
//
//     //   llVision.trackPose();
//
//        intake.ioTeleOp();
//
//        flywheelShooter.ioTeleOp();
//
//        // drive as usual, right bumper slowmode
//        // right trigger activates PD turning, can still strafe anywhere
//
//        drive.operateSimple();
//
//        /// full current cmds - all on gamepad 1. Drive is joysticks with right bumper slow mode
//        /// Intake is gamepad1 right trigger for manual control, gamepad1 a, b, and y for preprogrammed control
//        /// Shooter is dpad left for automatic after gamepad1 dpad up for 4700, down for 4000, and left for 0
//
//        // loop time measuring
////        telemetry.addData("Loop Times", elapsedtime.milliseconds());
////        elapsedtime.reset();
//
////        telemetryM.debug("variable: " + "variable.getValue()");
////        telemetryM.update(telemetry);
//    }
//}
