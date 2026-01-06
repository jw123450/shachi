//package org.firstinspires.ftc.teamcode.TeleOp.Testing;
//
//import com.bylazar.configurables.PanelsConfigurables;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.configurables.annotations.IgnoreConfigurable;
//import com.bylazar.field.FieldManager;
//import com.bylazar.field.PanelsField;
//import com.bylazar.field.Style;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.bylazar.telemetry.TelemetryManager;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//
//import org.firstinspires.ftc.teamcode.Subsystem.LimelightVision;
//import org.firstinspires.ftc.teamcode.Subsystem.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Subsystem.SubsystemTemplate;
//import org.firstinspires.ftc.teamcode.Util.RobotHardware;
//
//import java.util.List;
//
//@Configurable
//@TeleOp(name = "PD Turning Tuner", group = "Testing")
//public class TuningDrivePD extends OpMode{
//
//    private MecanumDrive drive = new MecanumDrive(); // TODO: motor + pinpoint config
//    RobotHardware robotHardware = new RobotHardware();
//    private ElapsedTime elapsedtime;
//    private List<LynxModule> allHubs;
//
//    @IgnoreConfigurable
//    static TelemetryManager telemetryM;
//
//    final Gamepad currentGamepad1 = new Gamepad();
//    final Gamepad currentGamepad2 = new Gamepad();
//    final Gamepad previousGamepad1 = new Gamepad();
//    final Gamepad previousGamepad2 = new Gamepad();
//
//    private
//
//    @Override
//    public void init() {
//        robotHardware.initialize(this);
//        drive.initialize(this, robotHardware);
//
//        // loop time stuff
//        elapsedtime = new ElapsedTime();
//        elapsedtime.reset();
//
//        // panels display
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        // bulk caching
//        allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
//    }
//
//    @Override
//    public void loop() {
//        for (LynxModule hub : allHubs) { hub.clearBulkCache(); }
//
//        //  rising edge detection
//        previousGamepad1.copy(currentGamepad1);
//        previousGamepad2.copy(currentGamepad2);
//        currentGamepad1.copy(gamepad1);
//        currentGamepad2.copy(gamepad2);
//
//        // drive as usual, right bumper slowmode
//        // right trigger activates PD turning, can still strafe anywhere
//        drive.operateTuningPD();
//
//        // loop time measuring
////        telemetry.addData("Loop Times", elapsedtime.milliseconds());
////        elapsedtime.reset();
//
//        telemetryM.debug("variable: " + "variable.getValue()");
//        telemetryM.update(telemetry);
//    }
//}