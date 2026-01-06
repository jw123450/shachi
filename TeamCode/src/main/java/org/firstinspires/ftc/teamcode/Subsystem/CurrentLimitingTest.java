//package org.firstinspires.ftc.teamcode.Subsystem;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//@TeleOp(name="MecanumDrive_SimulatedCurrentLimit", group="Testing")
//public class CurrentLimitingTest extends LinearOpMode{
//
//    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
//    private VoltageSensor battery;
//
//    // --- Motor parameters (for GoBILDA Yellow Jacket 13.7:1, 435 RPM @ 12V) ---
//    private static final double NOMINAL_VOLTAGE = 12.0;    // volts
//    private static final double FREE_SPEED_RPM = 435.0;    //
//    private static final double STALL_CURRENT = 5.2;       //
//    private static final double FREE_CURRENT = 0.2;        // amps
//
//    // Derived electrical constants
//    private static final double R = NOMINAL_VOLTAGE / STALL_CURRENT;   // ohms
//    private static final double OMEGA_FREE = (FREE_SPEED_RPM / 60.0) * 2.0 * Math.PI; // rad/s
//    private static final double KE = (NOMINAL_VOLTAGE - FREE_CURRENT * R) / OMEGA_FREE; // back-EMF constant
//
//    // Target current limit (amps)
//    private static final double CURRENT_LIMIT = 3.0;
//
//    @Override
//    public void runOpMode() {
//        frontLeft  = hardwareMap.get(DcMotorEx.class, "Fl");
//        frontRight = hardwareMap.get(DcMotorEx.class, "Fr");
//        backLeft   = hardwareMap.get(DcMotorEx.class, "Bl");
//        backRight  = hardwareMap.get(DcMotorEx.class, "Br");
//
//        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
//        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
//
//        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//
//        battery = hardwareMap.voltageSensor.iterator().next();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            double y  = -gamepad1.left_stick_y;  // forward/back
//            double x  =  gamepad1.left_stick_x;  // strafe
//            double rx =  gamepad1.right_stick_x; // rotation
//
//            // Basic mecanum mixing
//            double fl = y + x + rx;
//            double bl = y - x + rx;
//            double fr = y - x - rx;
//            double br = y + x - rx;
//
//            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
//                    Math.max(Math.abs(bl), Math.abs(br)));
//            if (max > 1.0) {
//                fl /= max; bl /= max; fr /= max; br /= max;
//            }
//
//            boolean limitMode = gamepad1.a;
//
//            double vBattery = battery.getVoltage();
//
//            if (limitMode) {
//                // Apply simulated current limiting
//                fl = limitPower(fl, vBattery);
//                fr = limitPower(fr, vBattery);
//                bl = limitPower(bl, vBattery);
//                br = limitPower(br, vBattery);
//            }
//
//            frontLeft.setPower(fl);
//            frontRight.setPower(fr);
//            backLeft.setPower(bl);
//            backRight.setPower(br);
//
//            telemetry.addData("Limiting", limitMode ? "ON" : "OFF");
//            telemetry.addData("Battery (V)", vBattery);
//            telemetry.update();
//        }
//    }
//
//    private double limitPower(double powerCmd, double vBattery) {
//        // Estimate motor speed as proportional to commanded power
//        double estOmega = Math.abs(powerCmd) * OMEGA_FREE;
//
//        // Compute back EMF
//        double vEmf = KE * estOmega;
//
//        // Compute actual voltage applied to motor
//        double vApplied = vBattery * Math.abs(powerCmd);
//
//        // Estimate current draw using I = (V - KE*ω)/R + free current
//        double estCurrent = ((vApplied - vEmf) / R) + FREE_CURRENT;
//
//        if (estCurrent > CURRENT_LIMIT) {
//            double scale = CURRENT_LIMIT / estCurrent;
//            powerCmd *= scale;
//        }
//        return powerCmd;
//    }
//}
//
