package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.PinpointManager;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


@Configurable
public class MecanumDrive {

    OpMode opmode;
    private DcMotorEx Fl, Fr, Bl, Br;
    private volatile double prevFrontLeftPower, prevBackLeftPower, prevFrontRightPower, prevBackRightPower;
    public PinpointManager pinpoint = new PinpointManager();
    ElapsedTime timer = new ElapsedTime();

    public static double SLOW_MODE_FACTOR = 0.5;
    public static double CACHING_THRESHOLD = 0.005;
    public static double SCALING_EXPONENT = 2.2;

    public volatile boolean angleLockBool = false, slowModeBool = false;

    public static double Kp = 0.0007; // TODO
    public static double Kd = 0.0001; // TODO

//    private VoltageSensor battery;

    // --- Motor parameters (for GoBILDA Yellow Jacket 13.7:1, 435 RPM @ 12V) ---
//    private static final double NOMINAL_VOLTAGE = 12.0;    // volts
//    private static final double FREE_SPEED_RPM = 435.0;    //
//    private static final double STALL_CURRENT = 9.2;       //
//    private static final double FREE_CURRENT = 0.25;        // amps
//
//    // Derived electrical constants
//    private static final double R = NOMINAL_VOLTAGE / STALL_CURRENT;   // ohms
//    private static final double OMEGA_FREE = (FREE_SPEED_RPM / 60.0) * 2.0 * Math.PI; // rad/s
//    private static final double KE = (NOMINAL_VOLTAGE - FREE_CURRENT * R) / OMEGA_FREE; // back-EMF constant

    // Target current limit (amps)
    private static final double CURRENT_LIMIT = 3.0;

    // PID
    public double targetAngle = 0;
    private double lastError;
    private List<Double> motorCurrents = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0, 0.0));

    public MecanumDrive() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;

        this.Fl = robotHardware.Fl;
        this.Fr = robotHardware.Fr;
        this.Bl = robotHardware.Bl;
        this.Br = robotHardware.Br;
    }

    public void operateTesting() {
        pinpoint.operateSimple();

        slowModeBool = opmode.gamepad1.left_trigger > 0.1;
        driveRobotCentric(opmode.gamepad1.left_stick_x, -opmode.gamepad1.left_stick_y, opmode.gamepad1.right_stick_x, slowModeBool);

        // gyro reset
        if (opmode.gamepad1.b) {pinpoint.softResetYaw();}

        // sets target
        if (opmode.gamepad1.a) {targetAngle = pinpoint.relativeNormalizedHeading;}

        opmode.telemetry.addData("Current Scaling Exponent: input^", SCALING_EXPONENT);
        opmode.telemetry.addData("rel normalized heading: ", pinpoint.relativeNormalizedHeading);
        opmode.telemetry.addData("normalized heading: ", pinpoint.normalizedHeading);
        opmode.telemetry.addData("target heading: ", targetAngle);
        opmode.telemetry.addData("gyro offset: ", pinpoint.offset);
        opmode.telemetry.addData("PD calculated rx [-1,1]: ", PDTurning(targetAngle, pinpoint.relativeNormalizedHeading));
        opmode.telemetry.addData("normalized error: ", normalizeError(targetAngle - pinpoint.relativeNormalizedHeading));
    }

//    public void operateTuningPD() {
//        // changing Kp and Kd values should already update globally
//        // just get heading
////        double tAngle = ll3a.getTargetAngle();
//
//        pinpoint.operateSimple();
//
//        // TODO: PD TURNING NEEDS TO HAVE ENOUGH POWER TO LOCK ONTO APRIL TAG WHEN SLOWMODE
//        if (opmode.gamepad1.right_trigger > 0.1){
////            driveRobotCentric(opmode.gamepad1.left_stick_x, -opmode.gamepad1.left_stick_y, PDTurning(tAngle, pinpoint.relativeNormalizedHeading), opmode.gamepad1.right_bumper);
//        } else {
//            operateSimple();
//        }
//
//        if (opmode.gamepad1.dpad_down) {
//            setTargetToCurrentHeading();
//        }
//
//        // not very necessary since running robot centric, but might as well include it
//        opmode.telemetry.addData("rel normalized heading: ", pinpoint.relativeNormalizedHeading);
//        opmode.telemetry.addData("gyro offset: ", pinpoint.offset);
//        opmode.telemetry.addData("normalized heading: ", pinpoint.normalizedHeading);
////        opmode.telemetry.addData("target heading: ", tAngle);
//        opmode.telemetry.addData("PD calculated rx [-1,1]: ", getPDPower());
////        opmode.telemetry.addData("Tx: ", ll3a.getTx());
////        opmode.telemetry.addData("normalized error:", normalizeError(tAngle - pinpoint.relativeNormalizedHeading));
//    }

    public void operateSimple() {
        if (timer.milliseconds() > 100) {
            readCurrents();
            timer.reset();
        }
        driveRobotCentric(opmode.gamepad1.left_stick_x, -opmode.gamepad1.left_stick_y, opmode.gamepad1.right_stick_x, opmode.gamepad1.right_stick_button || opmode.gamepad1.left_trigger > 0.3);
        opmode.telemetry.addData("Motor Currents (FL,FR,BL,BR): ", motorCurrents);
    }

    public void driveRobotCentric(double x, double y, double rx, boolean slowmode) {
        x = x * (slowmode ? SLOW_MODE_FACTOR * 0.6 : 0.67);
        y = y * (slowmode ? SLOW_MODE_FACTOR: 1);
        rx = rx * (slowmode ? SLOW_MODE_FACTOR * 0.6 : 1);
        // calculating output

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // current limiting
        frontLeftPower = limitPower(frontLeftPower, 0);
        frontRightPower = limitPower(frontRightPower, 1);
        backLeftPower = limitPower(backLeftPower, 2);
        backRightPower = limitPower(backRightPower, 3);

        // very un-wrappered power caching
        if (comparePower(prevFrontLeftPower, frontLeftPower)) { Fl.setPower(frontLeftPower); }
        if (comparePower(prevBackLeftPower, backLeftPower)) { Bl.setPower(backLeftPower); }
        if (comparePower(prevFrontRightPower, frontRightPower)) { Fr.setPower(frontRightPower); }
        if (comparePower(prevBackRightPower, backRightPower)) { Br.setPower(backRightPower); }

        // assigns for next loop
        prevFrontLeftPower = frontLeftPower;
        prevBackLeftPower = backLeftPower;
        prevFrontRightPower = frontRightPower;
        prevBackRightPower = backRightPower;
    }

    public void operateFieldCentric(double heading) {
        if (timer.milliseconds() > 100) {
            readCurrents();
            timer.reset();
        }
        driveFieldCentric(opmode.gamepad1.left_stick_x, -opmode.gamepad1.left_stick_y, opmode.gamepad1.right_stick_x, heading, opmode.gamepad1.right_stick_button || opmode.gamepad1.left_bumper);
        opmode.telemetry.addData("Motor Currents (FL,FR,BL,BR): ", motorCurrents);
    }

    public void driveFieldCentric(double x, double y, double rx, double heading, boolean slowmode) {
        x = x * (slowmode ? SLOW_MODE_FACTOR * 0.6 : 0.6);
        y = y * (slowmode ? SLOW_MODE_FACTOR: 1);
        rx = rx * (slowmode ? SLOW_MODE_FACTOR * 0.6 : 0.7);
        // calculating output
        double headingRads = Math.PI-Math.toRadians(heading);
        double rotX = y * Math.cos(headingRads) + x * Math.sin(headingRads);
        double rotY = y * Math.sin(headingRads) - x * Math.cos(headingRads);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // very un-wrappered power caching
        if (comparePower(prevFrontLeftPower, frontLeftPower)) { Fl.setPower(frontLeftPower); }
        if (comparePower(prevBackLeftPower, backLeftPower)) { Bl.setPower(backLeftPower); }
        if (comparePower(prevFrontRightPower, frontRightPower)) { Fr.setPower(frontRightPower); }
        if (comparePower(prevBackRightPower, backRightPower)) { Br.setPower(backRightPower); }

        // assigns for next loop
        prevFrontLeftPower = frontLeftPower;
        prevBackLeftPower = backLeftPower;
        prevFrontRightPower = frontRightPower;
        prevBackRightPower = backRightPower;
    }



    // checks if powers are different enough
    public boolean comparePower(double prevPower, double currentPower) {
        return Math.abs(currentPower - prevPower) >= CACHING_THRESHOLD;
    }

    public double PDTurning(double targetHeading, double currentHeading) {
        // calculate the error
        double error = normalizeError(currentHeading - targetHeading);

        double derivative = (error - lastError) / timer.seconds();

        double output = (Kp * error) + (Kd * derivative);
//        output = Math.signum(output) * Math.sqrt(Math.abs(output));
        output = Math.max(-1, Math.min(1, output));
        // square root PID, if robot is too fat and has too much inertia to fix small error
        // FLOAT mode might make tuning very hard :noooo:, since it could depend on robot's strafe movement, and a lot on mass
        // double output = Math.signum(output) * Math.min(1, Math.sqrt(Math.abs(output)));
        // double output = Math.signum(output) * Math.min(1, Math.pow(Math.abs(output), 0.7)); // 0.3-0.7 are all options too depends on robot fatness

        // reset stuff for next time
        timer.reset();
        lastError = error;

        return output;
    }

    private double normalizeError(double error) {
        // Normalize error to -180 to 180
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

//    public double scaleJoystick(double input) {
//        return Math.signum(input) * Math.pow(Math.abs(input), SCALING_EXPONENT);
//    }
//
//    public void setTargetToCurrentHeading() {
//        this.targetAngle = pinpoint.relativeNormalizedHeading;
//    }
//
//    public void setTargetAngle(double angle) {
//        this.targetAngle = angle;
//    }
//
//    public double getPDPower() {
//        return PDTurning(targetAngle, pinpoint.relativeNormalizedHeading);
//    }

    private void readCurrents() {
        motorCurrents.set(0, Fl.getCurrent(CurrentUnit.AMPS));
        motorCurrents.set(1, Fr.getCurrent(CurrentUnit.AMPS));
        motorCurrents.set(2, Bl.getCurrent(CurrentUnit.AMPS));
        motorCurrents.set(3, Br.getCurrent(CurrentUnit.AMPS));
    }

    // unfortunately allows current to spike for a single loop, but should be negligible
    private double limitPower(double powerCmd, int i) {
        double pulledCurrent = motorCurrents.get(i);
        if (pulledCurrent > CURRENT_LIMIT) {
            double scale = CURRENT_LIMIT / pulledCurrent;
            powerCmd *= scale;
        }
        return powerCmd;
    }
}