package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.Globals;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;


@Config
public class Shooter {
    OpMode opmode;
    DcMotorEx ShooterR, ShooterL;
    Servo hoodAngleAdjust, shooterLatch;

    // RPM targets
//    public static double MINIMUM_RPM = 2000;
//    public static double MAXIMUM_RPM = 4500;
    public static double IDLE_NEAR_RPM = 1800;
    public static double IDLE_FAR_RPM = 2900;
    public static double AT_RPM_RANGE = 50;

    // PID constants
    public static double kV = 0.000181716; // 1/6000
    public static double kP = 0.004;
    public static double kS = 0;

    // hood angle adjust
    public static double HOOD_ANGLE_MAX_POS = 0.979;
    public static double HOOD_ANGLE_MIN_POS = 0.4415;
    public static double MAX_ANGLE_DEG = 74; // lob, lanch angle in degrees above horizontal
    public static double MIN_ANGLE_DEG = 43; // laser

    // temporary for tuning loop
    public static double TEMP_LAUNCH_ANGLE = 74;
    public static double TEMP_RPM = 0;

    // servo latch
    public static double TUNING_INCREMENT = 0.001;
    public static double LATCH_OPEN_POS = 0.001;
    public static double LATCH_CLOSED_POS = 0.5;

    // REGRESSION CONSTANTS
    // from desmos
    double a1 = 1777.36447;
    double b1 = 0.0260802;
    double c1 = 2.78414;
    double d1 = 1619.40379;
    double a2 = 59.23412;
    double b2 = -0.022101;
    double c2 = -0.771555;

    // constantly changing variables
    public volatile double targetRPM = 0;
    public volatile double targetHoodAngle = 50;
    private volatile double output = 0;
    public volatile boolean latchOpen = false;
    public volatile boolean autoRPMmode = false;
    public volatile boolean atTargetRPM = false;

    public Shooter() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        this.ShooterR = robotHardware.ShooterR;
        this.ShooterL = robotHardware.ShooterL;
        this.hoodAngleAdjust = robotHardware.hoodAngleAdjust;
        this.shooterLatch = robotHardware.shooterLatch;

        ShooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ShooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        ShooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void operateTuning(TelemetryPacket packet) {
        double currentRPM = getCurrentRPM();

//        if (opmode.gamepad1.dpad_up)        { targetRPM = 2500; }
//        else if (opmode.gamepad1.dpad_down) { targetRPM = 1800; }
//        else if (opmode.gamepad1.dpad_left) { targetRPM = 0; }
//        else if (opmode.gamepad1.dpad_right){ targetRPM = TEMP_RPM;}

        targetRPM = TEMP_RPM;


        if (opmode.gamepad1.left_trigger > 0.2) {
            output = update(currentRPM);
            ShooterR.setPower(output);
            ShooterL.setPower(output);
        } else if (Math.abs(ShooterR.getPower()) > 0.02) {
            ShooterR.setPower(0);
            ShooterL.setPower(0);
        }

        atTargetRPM = Math.abs(currentRPM - targetRPM) < AT_RPM_RANGE;

        // graphing
        packet.put("power", ShooterR.getPower());
        packet.put("output", output);
        packet.put("current (amps)", ShooterR.getCurrent(CurrentUnit.AMPS));
        packet.put("currentRPM", currentRPM);
        packet.put("targetRPM", targetRPM);

        // shooter latch
        if (opmode.gamepad1.y) {
//            incremental(shooterLatch, 1);
            openLatch();
        } else if (opmode.gamepad1.a) {
//            incremental(shooterLatch, -1);
            closeLatch();
        }

        // hood adjust
//        if (opmode.gamepad1.b) {
////            hoodAngleAdjust.setPosition(HOOD_ANGLE_MAX_POS);
//            incremental(hoodAngleAdjust, 1);
//        } else if (opmode.gamepad1.x) {
////            hoodAngleAdjust.setPosition(HOOD_ANGLE_MIN_POS);
//            incremental(hoodAngleAdjust, -1);
//        }
        hoodAngleAdjust.setPosition(targetAngleToServoPos(TEMP_LAUNCH_ANGLE));

        opmode.telemetry.addData("hood adjust pos ", hoodAngleAdjust.getPosition());
        opmode.telemetry.addData("latch pos ", shooterLatch.getPosition());
        opmode.telemetry.addData("TEMP_LAUNCH_ANGLE", TEMP_LAUNCH_ANGLE);
        opmode.telemetry.addData("TEMP_RPM", TEMP_RPM);
        opmode.telemetry.addData("currentRPM", currentRPM);
        opmode.telemetry.addData("kP", kP);
        opmode.telemetry.addData("kV", kV);
        opmode.telemetry.addData("kS", kS);
    }

    public void operateFindMaxRPM() {
        double power = -opmode.gamepad1.left_stick_y;
        ShooterR.setPower(power);
        ShooterL.setPower(power);

        opmode.telemetry.addData("GP1 left stick Y: ", power);
        opmode.telemetry.addData("Current RPM     : ", ticksPerSecToRPM(ShooterR.getVelocity()));
        opmode.telemetry.addData("Amps pulled     : ", ShooterR.getCurrent(CurrentUnit.AMPS));
    }

    public void operateBasic(double currentX, double currentY, boolean blueAlliance, boolean vinWantsToShoot, boolean cyclingFarZone) {
        double currentRPM = getCurrentRPM();
        // calculate distance
        double x_dist = (blueAlliance ? Globals.blueGoalX : Globals.redGoalX) - currentX;
        double y_dist = (blueAlliance ? Globals.blueGoalY : Globals.redGoalY) - currentY;
        double dist = Math.hypot(x_dist, y_dist);

        if (autoRPMmode) {
            if (vinWantsToShoot) {
                targetRPM = distanceToRPM(dist);
                targetHoodAngle = distanceToHoodAngle(dist);
                hoodAngleAdjust.setPosition(targetAngleToServoPos(targetHoodAngle));
            } else if (cyclingFarZone) {
                targetRPM = IDLE_FAR_RPM;
            } else {
                targetRPM = IDLE_NEAR_RPM;
            }
        }

        output = update(currentRPM);

        // kill switch
        if (opmode.gamepad2.y) {
            autoRPMmode = false;
            output = 0;
            targetRPM = 0;
        }

        ShooterR.setPower(output);
        ShooterL.setPower(output);

        atTargetRPM = Math.abs(currentRPM - targetRPM) < AT_RPM_RANGE;

//        opmode.telemetry.addData("x_dist", x_dist);
//        opmode.telemetry.addData("y_dist", y_dist);
//        opmode.telemetry.addData("ShooterR amps pulled: ", ShooterR.getCurrent(CurrentUnit.AMPS));
//        opmode.telemetry.addData("ShooterL amps pulled: ", ShooterL.getCurrent(CurrentUnit.AMPS));
        opmode.telemetry.addLine("\nSHOOTER");
        opmode.telemetry.addData("dist", dist);
        opmode.telemetry.addData("shooter targetRPM", targetRPM);
        opmode.telemetry.addData("shooter currentRPM", currentRPM);
//        opmode.telemetry.addData("calculated RPM", distanceToRPM(dist));
    }

    public void operateSWM(boolean vinWantsToShoot, double adjustedDist, boolean cyclingFarZone) {
        double currentRPM = getCurrentRPM();
        if (autoRPMmode) {
            targetHoodAngle = distanceToHoodAngle(adjustedDist);
            hoodAngleAdjust.setPosition(targetAngleToServoPos(targetHoodAngle));
            if (vinWantsToShoot) {
                targetRPM = distanceToRPM(adjustedDist);
            } else if (cyclingFarZone) {
                targetRPM = IDLE_FAR_RPM;
            } else {
                targetRPM = IDLE_NEAR_RPM;
            }
        }

        output = update(currentRPM);

        // kill switch
        if (opmode.gamepad2.y) {
            autoRPMmode = false;
            output = 0;
            targetRPM = 0;
        }

        ShooterR.setPower(output);
        ShooterL.setPower(output);

        atTargetRPM = Math.abs(currentRPM - targetRPM) < AT_RPM_RANGE;

        opmode.telemetry.addLine("\nSHOOTER");
        opmode.telemetry.addData("shooter targetRPM", targetRPM);
        opmode.telemetry.addData("shooter currentRPM", currentRPM);

    }

    public void operateSWMSimple(double x_dist, double y_dist, boolean vinWantsToShoot, boolean cyclingFarZone) {
        double currentRPM = getCurrentRPM();
        // calculate distance
        double dist = Math.hypot(x_dist, y_dist);

        if (autoRPMmode) {
            if (vinWantsToShoot) {
                targetRPM = distanceToRPM(dist);
                targetHoodAngle = distanceToHoodAngle(dist);
                hoodAngleAdjust.setPosition(targetAngleToServoPos(targetHoodAngle));
            } else if (cyclingFarZone) {
                targetRPM = IDLE_FAR_RPM;
            } else {
                targetRPM = IDLE_NEAR_RPM;
            }
        }

        output = update(currentRPM);

        // kill switch
        if (opmode.gamepad2.right_bumper && opmode.gamepad2.y) {
            autoRPMmode = false;
            output = 0;
            targetRPM = 0;
        }

        ShooterR.setPower(output);
        ShooterL.setPower(output);

        atTargetRPM = Math.abs(currentRPM - targetRPM) < AT_RPM_RANGE;

        opmode.telemetry.addLine("\nSHOOTER");
        opmode.telemetry.addData("dist", dist);
        opmode.telemetry.addData("shooter targetRPM", targetRPM);
        opmode.telemetry.addData("shooter currentRPM", currentRPM);
//        opmode.telemetry.addData("calculated RPM", distanceToRPM(dist));
    }

    public void operateAuto(double currentX, double currentY, boolean blueAlliance, boolean runShooter, boolean farZone) {
        double currentRPM = getCurrentRPM();
        // calculate distance
        double x_dist = (blueAlliance ? Globals.blueGoalX : Globals.redGoalX) - currentX;
        double y_dist = (blueAlliance ? Globals.blueGoalY : Globals.redGoalY) - currentY;
        double dist = Math.hypot(x_dist, y_dist);

        if (runShooter) {
            targetRPM = distanceToRPM(dist);
            targetHoodAngle = distanceToHoodAngle(dist);
            hoodAngleAdjust.setPosition(targetAngleToServoPos(targetHoodAngle));
        } else if (farZone) {
            targetRPM = IDLE_FAR_RPM;
        } else {
            targetRPM = IDLE_NEAR_RPM;
        }

        output = update(currentRPM);

        ShooterR.setPower(output);
        ShooterL.setPower(output);

        atTargetRPM = Math.abs(currentRPM - targetRPM) < AT_RPM_RANGE;

        opmode.telemetry.addLine("\nSHOOTER");
        opmode.telemetry.addData("dist", dist);
        opmode.telemetry.addData("shooter targetRPM", targetRPM);
        opmode.telemetry.addData("shooter currentRPM", currentRPM);
    }

    public double update(double currentRPM) {
        double power = (kV * targetRPM) + (kP * (targetRPM - currentRPM));

        // Clamp motor power to safe range
        return Math.max(-0.1, Math.min(1, power));
    }

    public double getCurrentRPM() {
        return ticksPerSecToRPM(ShooterR.getVelocity());
    }


    private double ticksPerSecToRPM(double ticksPerSec) {
        return (ticksPerSec / 28.0) * 60.0; // tick/sec -> rev/sec -> rev/min
    }

    private double targetAngleToServoPos(double targetAngleDegrees) {
        // ex: target release angle = 45°
        // let MIN_SERVO_POS = 0.5 corresponds to 0° (horizontal)
        // let MAX_SERVO_POS = 0.1 corresponds to 90° (vertical)
        // corresponding hood angle = 0.75

        // Clamp input to within min/max angles already done in below distanceToHoodAngle() method
//        targetAngleDegrees = Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, targetAngleDegrees));

        // Linear interpolation
        double fraction = (targetAngleDegrees - MIN_ANGLE_DEG) /
                (MAX_ANGLE_DEG - MIN_ANGLE_DEG);

        return HOOD_ANGLE_MIN_POS + fraction * (HOOD_ANGLE_MAX_POS - HOOD_ANGLE_MIN_POS);
    }

    public void incremental(Servo s, int sign) {
        s.setPosition(s.getPosition() + sign * TUNING_INCREMENT);
    }

    public void openLatch() {
        shooterLatch.setPosition(LATCH_OPEN_POS);
        latchOpen = true;
    }

    public void closeLatch() {
        shooterLatch.setPosition(LATCH_CLOSED_POS);
        latchOpen = false;
    }

    public void initHood() {
        hoodAngleAdjust.setPosition(HOOD_ANGLE_MAX_POS);
    }

    public double distanceToRPM(double distance) {
        return (a1 / (1 + Math.exp(-(b1 * distance - c1)))) + d1;//regression equation
    }

    public double distanceToHoodAngle(double distance) {
        double angle = (a2 / (1 + Math.exp(-(b2 * distance - c2)))) + MIN_ANGLE_DEG;// regression equation

        return Math.max(Math.min(MAX_ANGLE_DEG, angle), MIN_ANGLE_DEG);
    }
}
