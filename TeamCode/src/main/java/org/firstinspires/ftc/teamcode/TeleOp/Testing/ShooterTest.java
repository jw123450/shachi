///*
//package org.firstinspires.ftc.teamcode.TeleOp.Testing;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
////import org.firstinspires.ftc.teamcode.Subsystem.MecanumDrive;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//
//import org.firstinspires.ftc.teamcode.Util.RobotHardware;
//
//import java.util.List;
//
//@Configurable
//@TeleOp(name = "Shooter Test", group = "Testing")
//public class ShooterTest extends OpMode {
//    //OpMode opmode;
//    private RobotHardware robotHardware = new RobotHardware();
//    private ElapsedTime elapsedtime;
//    private List<LynxModule> allHubs;
//    private volatile double ShooterPower;
//
//    private DcMotorEx S1, S2;
////    private double targetAngle;
//    private volatile double rpm;
////    private volatile double tps;
//
//
//
//    @Override
//    public void init() {
//        robotHardware.initialize(this);
////        S1 = opmode.hardwareMap.get(DcMotorEx.class, "S1");
////        S2 = opmode.hardwareMap.get(DcMotorEx.class, "S2");
//        // loop time stuff
//        elapsedtime = new ElapsedTime();
//        elapsedtime.reset();
//
//        // panels display
////        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        //activate LL poling
//        // bulk caching
//        allHubs = hardwareMap.getAll(LynxModule.class);
////        for (LynxModule hub : allHubs) {
////            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
////        }
//        this.S1 = robotHardware.S1;
//        this.S2 = robotHardware.S2;
//        S1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        S1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }
//
//
//    @Override
//    public void loop() {
////        for (LynxModule hub : allHubs) {
////            hub.clearBulkCache();
////        }
//
//        ShooterPower = gamepad1.left_stick_y;
//        S1.setPower(ShooterPower);
//        S2.setPower(ShooterPower);
//
//        if (gamepad1.a) {
//            ShooterPower = 0.66;
//            S1.setPower(ShooterPower);
//            S2.setPower(ShooterPower);
//        }
//
//        if (gamepad1.b) {
//            ShooterPower = 0.75;
//            S1.setPower(ShooterPower);
//            S2.setPower(ShooterPower);
//        }
//
//
//        rpm = S1.getVelocity() * 2.14; // 60/28
//
//        telemetry.addData("RPM: ", rpm);
//        telemetry.addData("Shooter Power: ", ShooterPower);
//        telemetry.update();
//
//        // loop time measuring
////        telemetry.addData("Loop Times", elapsedtime.milliseconds());
////        elapsedtime.reset();
//
////        telemetryM.debug("variable: " + "variable.getValue()");
////        telemetryM.update(telemetry);
//    }
//}*/
