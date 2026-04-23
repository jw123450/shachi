package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.Util.Globals;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Config
public class Turret {
    OpMode opmode;
    Servo leftServo, rightServo;
    public AbsoluteAnalogEncoder rightServoEnc;

    public static double CCW_LIMIT = -10.6443; // code limit
    public static double CW_LIMIT = 6.9486; // code limit
    public static double AT_TARGET_RANGE = 5; // in degrees
    public static double SERVO_MAX_RANGE = 342.4071;
    public static double TUNING_INCREMENT = 0.001;

    public static double Kv = 0; /// TODO later

    public static double temp_target = 180;

    // changing variables
    public double turretTargetAngle = 180;
    public boolean targetInRange = true;
    public boolean atTargetAngle = true;

    public Turret() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        this.leftServo = robotHardware.leftTurretServo;
        this.rightServo = robotHardware.rightTurretServo;
        rightServoEnc = new AbsoluteAnalogEncoder(opmode, robotHardware);
        leftServo.scaleRange(0, 0.976); // mathmatically should be 0.9689? (according to chatgpt)
        rightServo.scaleRange(0, 0.976);
//        ((PwmControl) leftServo).setPwmRange(new PwmControl.PwmRange(500, 24000));
//        ((PwmControl) rightServo).setPwmRange(new PwmControl.PwmRange(500, 24000));
    }

    public void operateTesting(TelemetryPacket packet, double chassisNormalizedHeading) {
        double currentAngle = rightServoEnc.getCurrentTurretAngle();
        double true_target_heading = temp_target;
        turretTargetAngle = normalize(true_target_heading - chassisNormalizedHeading);

        targetInRange = turretTargetAngle <= CCW_LIMIT || turretTargetAngle >= CW_LIMIT;
        atTargetAngle = Math.abs(currentAngle - turretTargetAngle) < AT_TARGET_RANGE;

        if (targetInRange && opmode.gamepad1.left_bumper) {
            setBoth(angleToServoPos(turretTargetAngle + opmode.gamepad1.right_stick_x * Kv));
        }
//        else if (opmode.gamepad1.x) {
//            setBoth(0);
//        }
//        else if (opmode.gamepad1.b) {
//            setBoth(1);
//        }
        else if (opmode.gamepad1.b) {
            setBoth(0.5);
        }

        if (opmode.gamepad1.dpad_left) {
            incremental(leftServo,1);
            incremental(rightServo,1);
        } else if (opmode.gamepad1.dpad_right) {
            incremental(leftServo,-1);
            incremental(rightServo,-1);
        }


        packet.put("current angle", currentAngle);
        packet.put("assigned pos", leftServo.getPosition());
        packet.put("target angle", turretTargetAngle);
        opmode.telemetry.addData("chassisNormalizedHeading", chassisNormalizedHeading);
        opmode.telemetry.addData("right stick X", opmode.gamepad1.right_stick_x);
        opmode.telemetry.addData("R assigned pos", rightServo.getPosition());
        opmode.telemetry.addData("L assigned pos", leftServo.getPosition());
        opmode.telemetry.addData("current angle", currentAngle);
        opmode.telemetry.addData("targetInRange", targetInRange);
        opmode.telemetry.addData("target angle", turretTargetAngle);
        opmode.telemetry.addData("atTargetAngle", atTargetAngle);
        opmode.telemetry.addData("temp_target", temp_target);
    }

    public void operateBasic(double currentX, double currentY, double chassisNormalizedHeading, boolean blueAlliance, boolean vinWantsToShoot) {
        // calculate TRUE target heading
        double currentAngle = rightServoEnc.getCurrentTurretAngle();
        double x_dist = (blueAlliance ? Globals.blueGoalX : Globals.redGoalX) - currentX;
        double y_dist = (blueAlliance ? Globals.blueGoalY : Globals.redGoalY) - currentY;
        double true_target_heading = Math.toDegrees(Math.atan2(y_dist, x_dist));
        turretTargetAngle = normalize(true_target_heading - chassisNormalizedHeading);

        targetInRange = turretTargetAngle <= CCW_LIMIT || turretTargetAngle >= CW_LIMIT;
        atTargetAngle = Math.abs(currentAngle - turretTargetAngle) < AT_TARGET_RANGE;

        if (vinWantsToShoot && targetInRange) { // in code-limited range
            setBoth(angleToServoPos(turretTargetAngle + opmode.gamepad1.right_stick_x * Kv));
        }
        else { // out of code limited range, or vincent doesn't wanna shoot, so default to center
            turretTargetAngle = 180;
            setBoth(angleToServoPos(turretTargetAngle));
        }

        opmode.telemetry.addLine("\nTURRET");
        opmode.telemetry.addData("turretTargetAngle", turretTargetAngle);
        opmode.telemetry.addData("turretCurrentAngle", currentAngle);
        opmode.telemetry.addData("target in turret range?", targetInRange);
    }

    /// Keep in mind edge case where the robot is driving bakcwards, and would jump from -180 to 180 repeatedly (fix is increasing range: -200° to 200°)
    public void operateSWM(double turretAdjustedTargetAngle) {
        double currentAngle = rightServoEnc.getCurrentTurretAngle();
        turretTargetAngle = turretAdjustedTargetAngle;

        targetInRange = turretTargetAngle <= CCW_LIMIT || turretTargetAngle >= CW_LIMIT;
        atTargetAngle = Math.abs(currentAngle - turretTargetAngle) < AT_TARGET_RANGE;

        if (targetInRange) { // in code-limited range
            /// adfsadfasdfadsfasdfadsfasdfasdfasdfasdfasdfasdfasdfasdfas
        }
        else { // out of code limited range, or vincent doesn't wanna shoot, so default to center
            turretTargetAngle = 180;
            /// adfsadfasdfadsfasdfadsfasdfasdfasdfasdfasdfasdfasdfasdfas
        }

//        opmode.telemetry.addLine("\nTURRET");
//        opmode.telemetry.addData("turretTargetAngle", turretTargetAngle);
//        opmode.telemetry.addData("turretCurrentAngle", currentAngle);
//        opmode.telemetry.addData("target in turret range?", targetInRange);
    }

    public void operateSWMSimple(double adj_x_dist, double adj_y_dist, double chassisNormalizedHeading, boolean vinWantsToShoot) {
        double currentAngle = rightServoEnc.getCurrentTurretAngle();

        double true_target_heading = Math.toDegrees(Math.atan2(adj_y_dist, adj_x_dist));
        turretTargetAngle = normalize(true_target_heading - chassisNormalizedHeading);

        targetInRange = turretTargetAngle <= CCW_LIMIT || turretTargetAngle >= CW_LIMIT;
        atTargetAngle = Math.abs(currentAngle - turretTargetAngle) < AT_TARGET_RANGE;

        if (vinWantsToShoot && targetInRange) { // in code-limited range
            setBoth(angleToServoPos(turretTargetAngle + opmode.gamepad1.right_stick_x * Kv));
        }
        else { // out of code limited range, or vincent doesn't wanna shoot, so default to center
            turretTargetAngle = 180;
            setBoth(angleToServoPos(turretTargetAngle));
        }

        opmode.telemetry.addLine("\nTURRET");
        opmode.telemetry.addData("turretTargetAngle", turretTargetAngle);
        opmode.telemetry.addData("turretCurrentAngle", currentAngle);
    }

    public void operateAuto(double currentX, double currentY, double pedroHeadingDegrees, boolean blueAlliance, boolean runTurret) {
        // calculate TRUE target heading
        double currentAngle = rightServoEnc.getCurrentTurretAngle();
        double x_dist = (blueAlliance ? Globals.blueGoalX : Globals.redGoalX) - currentX;
        double y_dist = (blueAlliance ? Globals.blueGoalY : Globals.redGoalY) - currentY;
        double true_target_heading = Math.toDegrees(Math.atan2(y_dist, x_dist));
        turretTargetAngle = normalize(true_target_heading - normalize(pedroHeadingDegrees));

        targetInRange = turretTargetAngle <= CCW_LIMIT || turretTargetAngle >= CW_LIMIT;
        atTargetAngle = Math.abs(currentAngle - turretTargetAngle) < AT_TARGET_RANGE;

        if (runTurret && targetInRange) { // in code-limited range
            setBoth(angleToServoPos(turretTargetAngle + opmode.gamepad1.right_stick_x * Kv));
        }
        else { // out of code limited range, or doesn't wanna shoot, so default to center
            turretTargetAngle = 180;
            setBoth(angleToServoPos(turretTargetAngle));
        }

        opmode.telemetry.addLine("TURRET TELEMETRY");
        opmode.telemetry.addData("turretTargetAngle ", turretTargetAngle);
        opmode.telemetry.addData("turretCurrentAngle ", currentAngle);
        opmode.telemetry.addData("targetInRange?", targetInRange);
        opmode.telemetry.addLine("\n");
    }

    public void operateSWMAuto(double adj_x_dist, double adj_y_dist, double pedroHeadingDegrees, boolean runTurret) {
        double currentAngle = rightServoEnc.getCurrentTurretAngle();

        double true_target_heading = Math.toDegrees(Math.atan2(adj_y_dist, adj_x_dist));
        turretTargetAngle = normalize(true_target_heading - normalize(pedroHeadingDegrees));

        targetInRange = turretTargetAngle <= CCW_LIMIT || turretTargetAngle >= CW_LIMIT;
        atTargetAngle = Math.abs(currentAngle - turretTargetAngle) < AT_TARGET_RANGE;

        if (runTurret && targetInRange) { // in code-limited range
            setBoth(angleToServoPos(turretTargetAngle + opmode.gamepad1.right_stick_x * Kv));
        }
        else { // out of code limited range, or doesn't wanna shoot, so default to center
            turretTargetAngle = 180;
            setBoth(angleToServoPos(turretTargetAngle));
        }


        opmode.telemetry.addLine("\nTURRET");
        opmode.telemetry.addData("turretTargetAngle", turretTargetAngle);
        opmode.telemetry.addData("turretCurrentAngle", currentAngle);
    }

    public void setBoth(double pos) {
        leftServo.setPosition(pos);
        rightServo.setPosition(pos);
    }

    public double angleToServoPos(double angle) {
        return norm360(angle - CW_LIMIT) / SERVO_MAX_RANGE;
    }

    private double normalize(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double norm360(double angle) {
        while (angle > 360) angle -= 360;
        while (angle < 0) angle += 360;
        return angle;
    }

    public void incremental(Servo s, int sign) {
        s.setPosition(s.getPosition() + sign * TUNING_INCREMENT);
    }

    // 24000 -> 0.959183

}
