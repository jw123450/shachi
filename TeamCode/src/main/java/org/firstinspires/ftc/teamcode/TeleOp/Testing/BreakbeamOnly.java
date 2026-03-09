//package org.firstinspires.ftc.teamcode.TeleOp.Testing;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Util.RobotHardware;
//
//@TeleOp(name = "Breakbeam Only", group = "Testing")
//public class BreakbeamOnly extends OpMode{
//    private RobotHardware robotHardware = new RobotHardware();
//
//    @Override
//    public void init() {
//        robotHardware.initialize(this);
//    }
//
//    @Override
//    public void loop() {
//        telemetry.addData("intake", robotHardware.intakeBreakBeam.getVoltage());
//        telemetry.addData("middle", robotHardware.middleBreakBeam.getVoltage());
//        telemetry.addData("transfer", robotHardware.transferBreakBeam.getVoltage());
//    }
//}
