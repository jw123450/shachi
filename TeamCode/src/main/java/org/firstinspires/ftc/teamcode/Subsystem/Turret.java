package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.Util.Globals;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Config
public class Turret {
    OpMode opmode;
    Servo leftServo, rightServo;
    AbsoluteAnalogEncoder leftServoEnc;

    public static double LEFT_LIMIT = 177.5; // code limit, true 18 inch sizing box limit is 115°
    public static double RIGHT_LIMIT = -177.5; // code limit
    public static double AT_TARGET_RANGE = 5; // in degrees

    public static double Kv = 0;

    public static double temp_target = 0;

    // changing variables
    private double lastError;
    ElapsedTime timer = new ElapsedTime();
    private double output = 0;
    private double turretTargetAngle = 0;
    public boolean targetInRange = true;
    public boolean atTargetAngle = true;

    public Turret() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        this.leftServo = robotHardware.leftTurretServo;
        this.rightServo = robotHardware.rightTurretServo;
        leftServoEnc = new AbsoluteAnalogEncoder(opmode, robotHardware);
    }

    public void operateTesting(TelemetryPacket packet, double chassisNormalizedHeading) {
        double currentAngle = leftServoEnc.getCurrentTurretAngle();
        double true_target_heading = temp_target;
        turretTargetAngle = normalize(true_target_heading - chassisNormalizedHeading);

        targetInRange = turretTargetAngle > RIGHT_LIMIT && turretTargetAngle < LEFT_LIMIT;
        atTargetAngle = Math.abs(currentAngle - turretTargetAngle) < AT_TARGET_RANGE;

        if (targetInRange && opmode.gamepad1.left_bumper) {
            setBoth(angleToServoPos(turretTargetAngle + opmode.gamepad1.right_stick_x * Kv));
        }

        packet.put("current angle", currentAngle);
        packet.put("assigned pos", leftServo.getPosition());
        packet.put("target angle", turretTargetAngle);
        opmode.telemetry.addData("right stick X", opmode.gamepad1.right_stick_x);
        opmode.telemetry.addData("current angle", currentAngle);
        opmode.telemetry.addData("target angle", turretTargetAngle);
        opmode.telemetry.addData("temp_target", temp_target);
    }

    public void operateOdomTracking(double currentX, double currentY, double chassisNormalizedHeading, boolean blueAlliance, boolean vinWantsToShoot) {
        // calculate TRUE target heading
        double currentAngle = leftServoEnc.getCurrentTurretAngle();
        double x_dist = (blueAlliance ? Globals.blueGoalX : Globals.redGoalX) - currentX;
        double y_dist = (blueAlliance ? Globals.blueGoalY : Globals.redGoalY) - currentY;
        double true_target_heading = Math.toDegrees(Math.atan2(y_dist, x_dist));
        turretTargetAngle = normalize(true_target_heading - chassisNormalizedHeading);

        targetInRange = turretTargetAngle > RIGHT_LIMIT && turretTargetAngle < LEFT_LIMIT;
        atTargetAngle = Math.abs(currentAngle - turretTargetAngle) < AT_TARGET_RANGE;

        if (targetInRange && opmode.gamepad1.right_trigger > 0.3) { // in code-limited range
            setBoth(angleToServoPos(turretTargetAngle + opmode.gamepad1.right_stick_x * Kv));
        }
        else { // out of code limited range, or vincent doesn't wanna shoot, so default to center
            turretTargetAngle = 0;
            setBoth(angleToServoPos(turretTargetAngle + opmode.gamepad1.right_stick_x * Kv));
        }

        opmode.telemetry.addLine("\nTURRET");
        opmode.telemetry.addData("turretTargetAngle", turretTargetAngle);
        opmode.telemetry.addData("turretCurrentAngle", currentAngle);
        opmode.telemetry.addData("target in turret range?", targetInRange);
    }

    /// Keep in mind edge case where the robot is driving bakcwards, and would jump from -180 to 180 repeatedly (fix is increasing range: -200° to 200°)
    public void operateSWM(double turretAdjustedTargetAngle) {
        double currentAngle = leftServoEnc.getCurrentTurretAngle();
        turretTargetAngle = turretAdjustedTargetAngle;

        targetInRange = turretTargetAngle > RIGHT_LIMIT && turretTargetAngle < LEFT_LIMIT;
        atTargetAngle = Math.abs(currentAngle - turretTargetAngle) < AT_TARGET_RANGE;

        if (targetInRange) { // in code-limited range
            /// adfsadfasdfadsfasdfadsfasdfasdfasdfasdfasdfasdfasdfasdfas
        }
        else { // out of code limited range, or vincent doesn't wanna shoot, so default to center
            turretTargetAngle = 0;
            /// adfsadfasdfadsfasdfadsfasdfasdfasdfasdfasdfasdfasdfasdfas
        }

//        opmode.telemetry.addLine("\nTURRET");
//        opmode.telemetry.addData("turretTargetAngle", turretTargetAngle);
//        opmode.telemetry.addData("turretCurrentAngle", currentAngle);
//        opmode.telemetry.addData("target in turret range?", targetInRange);
    }

    public void operateSWMSimple(double x_dist, double y_dist, double chassisNormalizedHeading) {
        double currentAngle = leftServoEnc.getCurrentTurretAngle();

        double true_target_heading = Math.toDegrees(Math.atan2(y_dist, x_dist));
        turretTargetAngle = normalize(true_target_heading - chassisNormalizedHeading);

        targetInRange = turretTargetAngle > RIGHT_LIMIT && turretTargetAngle < LEFT_LIMIT;
        atTargetAngle = Math.abs(currentAngle - turretTargetAngle) < AT_TARGET_RANGE;

        if (targetInRange) { // in code-limited range
            /// adfsadfasdfadsfasdfadsfasdfasdfasdfasdfasdfasdfasdfasdfas
        }
        else { // out of code limited range, or vincent doesn't wanna shoot, so default to center
            turretTargetAngle = 0;
            /// adfsadfasdfadsfasdfadsfasdfasdfasdfasdfasdfasdfasdfasdfas
        }

        opmode.telemetry.addLine("\nTURRET");
        opmode.telemetry.addData("turretTargetAngle", turretTargetAngle);
        opmode.telemetry.addData("turretCurrentAngle", currentAngle);
    }

    public void operateAuto(double currentX, double currentY, double pedroHeadingDegrees, boolean blueAlliance, boolean runTurret) {
        // calculate TRUE target heading
        double currentAngle = leftServoEnc.getCurrentTurretAngle();
        double x_dist = (blueAlliance ? Globals.blueGoalX : Globals.redGoalX) - currentX;
        double y_dist = (blueAlliance ? Globals.blueGoalY : Globals.redGoalY) - currentY;
        double true_target_heading = Math.toDegrees(Math.atan2(y_dist, x_dist));
        turretTargetAngle = normalize(true_target_heading - normalize(pedroHeadingDegrees));

        targetInRange = turretTargetAngle > RIGHT_LIMIT && turretTargetAngle < LEFT_LIMIT;
        atTargetAngle = Math.abs(currentAngle - turretTargetAngle) < AT_TARGET_RANGE;

        if (targetInRange && runTurret) { // in code-limited range
            /// adfsadfasdfadsfasdfadsfasdfasdfasdfasdfasdfasdfasdfasdfas
        } else { // out of code limited range, so default to center
            turretTargetAngle = 0;
            /// adfsadfasdfadsfasdfadsfasdfasdfasdfasdfasdfasdfasdfasdfas
        }

        opmode.telemetry.addLine("TURRET TELEMETRY");
        opmode.telemetry.addData("turretTargetAngle ", turretTargetAngle);
        opmode.telemetry.addData("turretCurrentAngle ", currentAngle);
        opmode.telemetry.addData("targetInRange?", targetInRange);
        opmode.telemetry.addLine("\n");
    }

    public void setBoth(double pos) {
        leftServo.setPosition(pos);
        rightServo.setPosition(pos);
    }

    public double getAssignedTurretAngle() { // relative to bot
        return 355*(leftServo.getPosition() - 0.5);
    }

    public double angleToServoPos(double angle) {
        // technically should limit input range, but will be taken care of somewhere else
        return (angle / 355) + 0.5;
    }

    private double normalize(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

}
