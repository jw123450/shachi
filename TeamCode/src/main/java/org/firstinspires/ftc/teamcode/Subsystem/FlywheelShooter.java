package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.Globals;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;
import org.firstinspires.ftc.teamcode.Util.RGBLights;


@Config
public class FlywheelShooter {
    OpMode opmode;
    DcMotorEx S1, S2;
    Servo shooterLatch;

    LimelightVision ll3a;
    PIDFController pidfController;
    private VoltageSensor battery;

    // Motor parameters
    private static final double NOMINAL_VOLTAGE = 12.0;   // volts
    private static final double FREE_SPEED_RPM = 5400;    // from gobilda page
    private static final double STALL_CURRENT = 9.2;      // amps
    private static final double FREE_CURRENT = 0.25;      // amps

    // Derived electrical constants
    private static final double R = NOMINAL_VOLTAGE / STALL_CURRENT;   // ohms
    private static final double OMEGA_FREE = (FREE_SPEED_RPM / 60.0) * 2.0 * Math.PI; // rad/s
    private static final double KE = (NOMINAL_VOLTAGE - FREE_CURRENT * R) / OMEGA_FREE; // back-EMF constant

    // Target current limit (amps)
    private static double CURRENT_LIMIT = 3.0;

    // Camera constants
    private static final double MIN_DIST = 36.0; // in inches
    private static final double MAX_DIST = 140.0; // in inches

    // RPM targets
    public static double FAR_RPM = 3100;
    public static double NEAR_RPM = 2650; // look at below for actual values
    public static double MINIMUM_RPM = 2000;
    public static double MAXIMUM_RPM = 3650;
    public static double ODOM_IDLE_NEAR_RPM = 2300;
    public static double AUTO_IDLE_NEAR_RPM = 2100; // specifially for auto
    public static double ODOM_IDLE_FAR_RPM = 3000;
    public static double AT_RPM_RANGE = 50;

    // odom dist to RPM calculation cosntants
    public static double MINIMUM_DISTANCE = 36.0;
    public static double MAXIMUM_DISTANCE = 193.6;
    public static double DISTANCE_CUTOFF = 157;

    // PID constants
    public static double kP = 0.0016;
    public static double kI = 0; // keep at 0
    public static double kD = 0.00001; // 0 – 0.001
    public static double kV = 0.0001766666667; // 1/6000

    // for if we have hood adjust later
    public static double HOOD_ANGLE_MAX_POS = 0; // servo pos 0-1
    public static double HOOD_ANGLE_MIN_POS = 0;
    public static double CORRESPONDING_MAX_ANGLE = 90; // degrees above horizontal
    public static double CORRESPONDING_MIN_ANGLE = 0;

    // servo latch
    public static double TUNING_INCREMENT = 0.001;
    public static double LATCH_OPEN_POS = 0.2;
    public static double LATCH_CLOSED_POS = 0.12;

    // constantly changing variables
    public volatile double targetRPM = 0;
    private volatile double output = 0;
    public volatile boolean shooterLatchOpen = false;
    public volatile boolean autoRPMmode = false;
    public volatile boolean atTargetRPM = false;

    public FlywheelShooter() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware, LimelightVision ll3a) {
        this.opmode = opmode;
        this.S1 = robotHardware.S1;
        this.S2 = robotHardware.S2;
        this.shooterLatch = robotHardware.shooterLatch;
        this.ll3a = ll3a;

        this.battery = robotHardware.battery;

        pidfController = new PIDFController(kP, kI, kD, kV);
        pidfController.setPIDF(kP, kI, kD, kV);

        S1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        S1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void operateOdomTracking(double currentX, double currentY, boolean blueAlliance, boolean vinWantsToShoot, boolean cyclingFarZone) {
        double currentRPM = getCurrentRPM();
        // calculate distance
        double x_dist = (blueAlliance ? Globals.blueGoalX : Globals.redGoalX) - currentX;
        double y_dist = (blueAlliance ? Globals.blueGoalY : Globals.redGoalY) - currentY;
        double dist = Math.sqrt(Math.pow(x_dist, 2) + Math.pow(y_dist, 2));

        if (autoRPMmode) {
            if (vinWantsToShoot) {
                targetRPM = odomDistanceToRPM(dist);
            } else if (cyclingFarZone) {
                targetRPM = ODOM_IDLE_FAR_RPM;
            } else {
                targetRPM = ODOM_IDLE_NEAR_RPM;
            }
        }

        output = update();

        // kill switch
        if (opmode.gamepad2.y) {
            autoRPMmode = false;
            output = 0;
            targetRPM = 0;
        }

        S1.setPower(output);
        S2.setPower(output);

        atTargetRPM = Math.abs(currentRPM - targetRPM) < AT_RPM_RANGE;

//        opmode.telemetry.addData("x_dist", x_dist);
//        opmode.telemetry.addData("y_dist", y_dist);
//        opmode.telemetry.addData("S1 amps pulled: ", S1.getCurrent(CurrentUnit.AMPS));
//        opmode.telemetry.addData("S2 amps pulled: ", S2.getCurrent(CurrentUnit.AMPS));
//        opmode.telemetry.addData("dist", dist);
        opmode.telemetry.addLine("\nSHOOTER");
        opmode.telemetry.addData("shooter targetRPM", targetRPM);
        opmode.telemetry.addData("shooter currentRPM", currentRPM);
        opmode.telemetry.addData("calculated RPM", odomDistanceToRPM(dist));
    }

    public void operateOdomAuto(double currentX, double currentY, boolean blueAlliance, boolean runShooter, boolean farZone) {
        double currentRPM = getCurrentRPM();
        // calculate distance
        double x_dist = (blueAlliance ? Globals.blueGoalX : Globals.redGoalX) - currentX;
        double y_dist = (blueAlliance ? Globals.blueGoalY : Globals.redGoalY) - currentY;
        double dist = Math.sqrt(Math.pow(x_dist, 2) + Math.pow(y_dist, 2));

        if (runShooter) {
            targetRPM = odomDistanceToRPM(dist);
        } else if (farZone) {
            targetRPM = ODOM_IDLE_FAR_RPM;
        } else {
            targetRPM = AUTO_IDLE_NEAR_RPM;
        }

        output = update();
        S1.setPower(output);
        S2.setPower(output);

        atTargetRPM = Math.abs(currentRPM - targetRPM) < AT_RPM_RANGE;

        opmode.telemetry.addLine("SHOOTER TELEMETRY");
        opmode.telemetry.addData("dist", dist);
        opmode.telemetry.addData("shooter targetRPM", targetRPM);
        opmode.telemetry.addData("shooter currentRPM", currentRPM);
        opmode.telemetry.addData("calculated RPM", odomDistanceToRPM(dist));
        opmode.telemetry.addLine("\n");
    }

    public void ioTeleOp() {
        if (opmode.gamepad1.dpad_up) {
            autoRPMmode = false;
            targetRPM = FAR_RPM;
        } else if (opmode.gamepad1.dpad_right) {
            autoRPMmode = true;
        }
        if (battery.getVoltage() > 12.5) {
            NEAR_RPM = 2600;
        } else if (NEAR_RPM != 2700) {
            NEAR_RPM = 2700;
        }

        atTargetRPM = Math.abs(getCurrentRPM() - targetRPM) < AT_RPM_RANGE;

        if (autoRPMmode) {
            targetRPM = llVelAdjust(this.ll3a.readDistance());
        }

        output = update();

        if (opmode.gamepad2.y) {
            autoRPMmode = false;
            output = 0;
            targetRPM = 0;
        }

        S1.setPower(output);
        S2.setPower(output);

        opmode.telemetry.addData("S1 power", S1.getPower());
        opmode.telemetry.addData("S2 power", S2.getPower());
//        opmode.telemetry.addData("shooter output", output);
        opmode.telemetry.addData("current limit (amps)", CURRENT_LIMIT);
        opmode.telemetry.addData("shooter currentRPM", ticksPerSecToRPM(S1.getVelocity()));
        opmode.telemetry.addData("shooter targetRPM", targetRPM);
        opmode.telemetry.addData("using camera for RPM? ", autoRPMmode);
        opmode.telemetry.addData("atTargetRPM", atTargetRPM);
    }

    public void kauaiTeleOp() {
        ioTeleOp();
    }

    public void operateTuning(TelemetryPacket packet) {
        pidfController.setPIDF(kP, kI, kD, kV);

        if (opmode.gamepad1.left_trigger > 0.2) {
            output = update();
            S1.setPower(output);
            S2.setPower(output);
        } else if (S1.getPower() > 0.02){
            S1.setPower(0);
            S2.setPower(0);
        }

        if (opmode.gamepad1.dpad_up)        { targetRPM = FAR_RPM; }
        else if (opmode.gamepad1.dpad_down) { targetRPM = NEAR_RPM; }
        else if (opmode.gamepad1.dpad_left) { targetRPM = 0; }

        atTargetRPM = Math.abs(getCurrentRPM() - targetRPM) < AT_RPM_RANGE;

        // graphing
        packet.put("power", S1.getPower());
        packet.put("output", output);
        packet.put("current (milliamps)", S1.getCurrent(CurrentUnit.MILLIAMPS));
        packet.put("currentRPM", getCurrentRPM());
        packet.put("targetRPM", targetRPM);

        if (opmode.gamepad1.y) {
            incremental(1);
        } else if (opmode.gamepad1.a) {
            incremental(-1);
        } else if (opmode.gamepad1.b) {
            openLatch();
        } else if (opmode.gamepad1.x) {
            closeLatch();
        }
//        opmode.telemetry.addData("servo latch pos: ", shooterLatch.getPosition());
//        opmode.telemetry.addData("latch open? ", shooterLatchOpen);
        opmode.telemetry.addData("currentRPM", getCurrentRPM());
        opmode.telemetry.addData("kP", kP);
        opmode.telemetry.addData("kD", kD);
        opmode.telemetry.addData("kV", kV);
    }

    public void operateFindMaxRPM() {
        S1.setPower(-opmode.gamepad1.left_stick_y);

        opmode.telemetry.addData("GP1 left stick Y: ", -opmode.gamepad1.left_stick_y);
        opmode.telemetry.addData("Current Power   : ", S1.getPower());
        opmode.telemetry.addData("Current RPM     : ", ticksPerSecToRPM(S1.getVelocity()));
        opmode.telemetry.addData("Amps pulled     : ", S1.getCurrent(CurrentUnit.AMPS));
    }

    public void operateAutoLL(boolean runShooter) {
        if (runShooter) {
            targetRPM = llVelAdjust(this.ll3a.readDistance());
            double vBattery = battery.getVoltage();

            output = update();
            S1.setPower(output);
            S2.setPower(output);
        } else {
            S1.setPower(0);
            S2.setPower(0);
        }

        atTargetRPM = Math.abs(getCurrentRPM() - targetRPM) < AT_RPM_RANGE;
    }

    public double update() {
        double currentRPM = ticksPerSecToRPM(S1.getVelocity()); // RPM
        double power = pidfController.calculate(currentRPM, targetRPM);

        // Clamp motor power to safe range
        return Math.max(-0.1, Math.min(1, power));
    }

    private double ticksPerSecToRPM(double ticksPerSec) {
        double ticksPerRev = 28.0; // for 6000 RPM motors
        return (ticksPerSec / ticksPerRev) * 60.0;
    }


    public double llVelAdjust(double llDistance) {

        double dist_ratio = (1 / (MAX_DIST - MIN_DIST)) * (Math.max(llDistance, MIN_DIST) - MIN_DIST);

        double deltaVelocity = (NEAR_RPM - MINIMUM_RPM);

        return MINIMUM_RPM + (dist_ratio * deltaVelocity);
    }

    private double targetAngleToServoPos(double targetAngleDegrees) {
        // ex: target release angle = 45°
        // let MIN_SERVO_POS = 0.5 corresponds to 0° (horizontal)
        // let MAX_SERVO_POS = 0.1 corresponds to 90° (vertical)
        // corresponding hood angle = 0.75

        // Clamp input to within min/max angles
        /// probably not necessary?
        targetAngleDegrees = Math.max(CORRESPONDING_MIN_ANGLE, Math.min(CORRESPONDING_MAX_ANGLE, targetAngleDegrees));

        // Linear interpolation
        double fraction = (targetAngleDegrees - CORRESPONDING_MIN_ANGLE) /
                (CORRESPONDING_MAX_ANGLE - CORRESPONDING_MIN_ANGLE);

        return HOOD_ANGLE_MIN_POS + fraction * (HOOD_ANGLE_MAX_POS - HOOD_ANGLE_MIN_POS);
    }

    public void incremental(int sign) {
        shooterLatch.setPosition(shooterLatch.getPosition() + sign * TUNING_INCREMENT);
    }

    public void openLatch() {
        shooterLatch.setPosition(LATCH_OPEN_POS);
        shooterLatchOpen = true;
    }

    public void closeLatch() {
        shooterLatch.setPosition(LATCH_CLOSED_POS);
        shooterLatchOpen = false;
    }

    public double getCurrentRPM() {
        return ticksPerSecToRPM(S1.getVelocity());
    }

    // unfortunately allows current to spike for a single loop, but should be negligible
    private double limitPower(double powerCmd, DcMotorEx motor) {
        double pulledCurrent = motor.getCurrent(CurrentUnit.AMPS);
        if (pulledCurrent > CURRENT_LIMIT) {
            double scale = CURRENT_LIMIT / pulledCurrent;
            powerCmd *= scale;
        }
        return powerCmd;
    }

    public double odomDistanceToRPM(double distance) {
        double dist_ratio = (1 / (MAXIMUM_DISTANCE - MINIMUM_DISTANCE)) * (Math.max(distance, MINIMUM_DISTANCE) - MINIMUM_DISTANCE);
        double deltaVelocity = MAXIMUM_RPM - MINIMUM_RPM;

        return MINIMUM_RPM + (dist_ratio * deltaVelocity);
    }
}
