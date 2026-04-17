package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


@Config
public class MecanumDrive {

    OpMode opmode;
    private DcMotorEx Fl, Fr, Bl, Br;
    private volatile double prevFrontLeftPower, prevBackLeftPower, prevFrontRightPower, prevBackRightPower;
    private ElapsedTime timer = new ElapsedTime();

    public static double SLOW_MODE_FACTOR = 0.5;
    public static double CACHING_THRESHOLD = 0.005; // TODO

    public static double Kp = 0.007; // TODO
    public static double Kd = 0.0001; // TODO
    public static double BLUE_GATE_HEADING = 140;
    public static double RED_GATE_HEADING = 40;

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

    public void operateTeleOp(double currentHeading, boolean blueAlliance) {
//        if (timer.milliseconds() > 100) {
//            readCurrents();
//            timer.reset();
//        }
        driveRobotCentric(opmode.gamepad1.left_stick_x, -opmode.gamepad1.left_stick_y, (opmode.gamepad1.a ? calculatePD((blueAlliance ? BLUE_GATE_HEADING : RED_GATE_HEADING), currentHeading) : opmode.gamepad1.right_stick_x),  opmode.gamepad1.left_trigger > 0.3);
//        opmode.telemetry.addData("Motor Currents (FL,FR,BL,BR): ", motorCurrents);

        if ((opmode.gamepad1.left_trigger > 0.3 || opmode.gamepad1.a) && Fl.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE) {
            Fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            Fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            Bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            Br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        } else if ((opmode.gamepad1.left_trigger < 0.3 && !opmode.gamepad1.a) && Fl.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE) {
            Fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            Fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            Bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            Br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }

        opmode.telemetry.addLine("\n DEBUGGING"); // TODO Comment out later
        opmode.telemetry.addData("currentHeading ", currentHeading);
        opmode.telemetry.addData("targetHeading ", blueAlliance ? BLUE_GATE_HEADING : RED_GATE_HEADING);
        opmode.telemetry.addData("blueAlliance", blueAlliance);
        opmode.telemetry.addData("calculated rx [-1,1] ", calculatePD(blueAlliance ? BLUE_GATE_HEADING : RED_GATE_HEADING, currentHeading));
    }

    public void driveRobotCentric(double x, double y, double rx, boolean slowmode) {
        x = x * (slowmode ? SLOW_MODE_FACTOR * 0.6 : 1);
        y = y * (slowmode ? SLOW_MODE_FACTOR: 1);
        rx = rx * (slowmode ? SLOW_MODE_FACTOR * 0.6 : 0.8);
        // calculating output

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // current limiting
//        frontLeftPower = limitPower(frontLeftPower, 0);
//        frontRightPower = limitPower(frontRightPower, 1);
//        backLeftPower = limitPower(backLeftPower, 2);
//        backRightPower = limitPower(backRightPower, 3);

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
        return Math.abs(currentPower - prevPower) > CACHING_THRESHOLD;
    }

    public double calculatePD(double targetHeading, double currentHeading) {
        // calculate the error
        double error = normalizeError(currentHeading - targetHeading);

        double derivative = (error - lastError) / timer.seconds();

        double output = (Kp * error) + (Kd * derivative); // TODO maybe need sqrt? (below)
        output = Math.signum(output) * Math.sqrt(Math.min(1, Math.abs(output)));
//        output = Math.max(-1, Math.min(1, output));

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

//    private void readCurrents() {
//        motorCurrents.set(0, Fl.getCurrent(CurrentUnit.AMPS));
//        motorCurrents.set(1, Fr.getCurrent(CurrentUnit.AMPS));
//        motorCurrents.set(2, Bl.getCurrent(CurrentUnit.AMPS));
//        motorCurrents.set(3, Br.getCurrent(CurrentUnit.AMPS));
//    }
//
//    // unfortunately allows current to spike for a single loop, but should be negligible
//    private double limitPower(double powerCmd, int i) {
//        double pulledCurrent = motorCurrents.get(i);
//        if (pulledCurrent > CURRENT_LIMIT) {
//            double scale = CURRENT_LIMIT / pulledCurrent;
//            powerCmd *= scale;
//        }
//        return powerCmd;
//    }
}