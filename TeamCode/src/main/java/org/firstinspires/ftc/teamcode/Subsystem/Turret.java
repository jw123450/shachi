package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.Globals;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Config
public class Turret {
    OpMode opmode;
    DcMotorEx turretMotor;
    LimelightVision ll3a;

    public static double LEFT_LIMIT = 115; // code limit, true 18 inch sizing box limit is 115°
    public static double RIGHT_LIMIT = -115; // code limit
    public static double AT_TARGET_RANGE = 5; // in degrees
    public static double DEGREES_PER_TICK = 72.0 / 384.5;
    public static double ANGLE_OFFSET = -2; // for less bouncing out

    // Motor parameters
    private static final double NOMINAL_VOLTAGE = 12.0;   // volts
    private static final double FREE_SPEED_RPM = 435;    // maybe
    private static final double STALL_CURRENT = 9.2;      // amps
    private static final double FREE_CURRENT = 0.25;      // amps

    // Derived electrical constants
    private static final double R = NOMINAL_VOLTAGE / STALL_CURRENT;   // ohms
    private static final double OMEGA_FREE = (FREE_SPEED_RPM / 60.0) * 2.0 * Math.PI; // rad/s
    private static final double KE = (NOMINAL_VOLTAGE - FREE_CURRENT * R) / OMEGA_FREE; // back-EMF constant

    // Target current limit (amps)
    private static double CURRENT_LIMIT = 3.0;

    private VoltageSensor battery;

    // PID constants
    public static double Kp = 0.007;
    public static double Kd = 0.0001;
    public static double POWER_CLAMP = 1;

    // changing variables
    private double lastError;
    ElapsedTime timer = new ElapsedTime();
    private double output = 0;
    private double turretTargetAngle = 0;
    public boolean useLL = true;
    public boolean use110 = false;
    public double nonLLSetAngle = 0;
    public boolean targetInRange = true;
    public boolean atTargetAngle = true;

    public Turret() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware, LimelightVision ll3a, boolean resetEncoders) {
        this.opmode = opmode;
        this.ll3a = ll3a;
        this.turretMotor = robotHardware.turretMotor;
        this.battery = robotHardware.battery;

        if (resetEncoders) {
            turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void operateTesting(TelemetryPacket packet, boolean blueAlliance) {
        if (timer.milliseconds() > 1000) { timer.reset(); }

        double currentAngle = getCurrentTurretAngle();
        boolean usePID = opmode.gamepad1.right_bumper;

        if (useLL && usePID) {
            if (ll3a.tagDetected(blueAlliance)) {
                turretTargetAngle = currentAngle + ll3a.getTx();
                output = update();
                if (currentAngle < LEFT_LIMIT && currentAngle > RIGHT_LIMIT) { // within hardstops, so good
                    turretMotor.setPower(output);
                } else if (currentAngle >= LEFT_LIMIT && output < 0) { // past left limit, but positive output, so good
                    turretMotor.setPower(output);
                } else if (currentAngle <= RIGHT_LIMIT && output > 0) { // past right limit, but negative output, so good
                    turretMotor.setPower(output);
                } else { // must be both outside range && target is also outside range, so power 0
                    turretMotor.setPower(0);
                }
            }
            else { // no april tag detected, so default to center
                turretTargetAngle = 0;
                output = update();
                turretMotor.setPower(output);
            }
        } else if (usePID) { // !useLL already implied
            turretTargetAngle = nonLLSetAngle;
            output = update();
            turretMotor.setPower(output);
        } else {
            turretMotor.setPower(0);
        }

        packet.put("power", turretMotor.getPower());
        packet.put("current (amps)", turretMotor.getCurrent(CurrentUnit.AMPS));
        packet.put("current angle", currentAngle);
        packet.put("target angle", turretTargetAngle);
        packet.put("Tx", ll3a.getTx());
        opmode.telemetry.addData("tagDetected", ll3a.tagDetected(blueAlliance));
        opmode.telemetry.addData("useLL", useLL);
        opmode.telemetry.addData("use110", use110);
        opmode.telemetry.addData("usePID", usePID);
        opmode.telemetry.addData("power", turretMotor.getPower());
        opmode.telemetry.addData("current angle", currentAngle);
        opmode.telemetry.addData("target angle", turretTargetAngle);
        opmode.telemetry.addData("Tx", ll3a.getTx()); // left of screen is positive
    }

    public void kauaiTeleOp(boolean blueAlliance) {
        double currentAngle = getCurrentTurretAngle();

        if (useLL) {
            if (ll3a.tagDetected(blueAlliance)) {
                turretTargetAngle = currentAngle + ll3a.getTx() + (blueAlliance ? ANGLE_OFFSET: -ANGLE_OFFSET);
                output = update();
                if (currentAngle < LEFT_LIMIT && currentAngle > RIGHT_LIMIT) { // within hardstops, so good
                    turretMotor.setPower(output);
                } else if (currentAngle >= LEFT_LIMIT && output < 0) { // past left limit, but positive output, so good
                    turretMotor.setPower(output);
                } else if (currentAngle <= RIGHT_LIMIT && output > 0) { // past right limit, but negative output, so good
                    turretMotor.setPower(output);
                } else { // must be both outside range && target is also outside range, so power 0
                    turretMotor.setPower(0);
                }
            }
            else { // no april tag detected, so default to center
                turretTargetAngle = 0;
                output = update();
                turretMotor.setPower(output);
            }
        } else {
            turretTargetAngle = nonLLSetAngle;
            output = update();
            turretMotor.setPower(output);
        }

        opmode.telemetry.addData("turret current angle ", currentAngle);
        opmode.telemetry.addData("turret target angle ", turretTargetAngle);
    }

    public void operateAutoLL(boolean runTurret, boolean blueAlliance) {
        double currentAngle = getCurrentTurretAngle();

        if (runTurret) {
            if (ll3a.tagDetected(blueAlliance)) {
                turretTargetAngle = currentAngle + ll3a.getTx() + (blueAlliance ? ANGLE_OFFSET: -ANGLE_OFFSET);
                output = update();
                if (currentAngle < LEFT_LIMIT && currentAngle > RIGHT_LIMIT) { // within hardstops, so good
                    turretMotor.setPower(output);
                } else if (currentAngle >= LEFT_LIMIT && output < 0) { // past left limit, but positive output, so good
                    turretMotor.setPower(output);
                } else if (currentAngle <= RIGHT_LIMIT && output > 0) { // past right limit, but negative output, so good
                    turretMotor.setPower(output);
                } else { // must be both outside range && target is also outside range, so power 0
                    turretMotor.setPower(0);
                }
            } else { // no april tag detected
                turretTargetAngle = blueAlliance ? -40 : 40; // TODO: tune backup angle
                output = update();
                turretMotor.setPower(output);
                opmode.telemetry.addLine("NO APRILTAG DETECTED!!!!!!!!!!!!!!!");
                opmode.telemetry.addLine("NO APRILTAG DETECTED!!!!!!!!!!!!!!!");
                opmode.telemetry.addLine("NO APRILTAG DETECTED!!!!!!!!!!!!!!!");
                opmode.telemetry.addLine("NO APRILTAG DETECTED!!!!!!!!!!!!!!!");
                opmode.telemetry.addLine("NO APRILTAG DETECTED!!!!!!!!!!!!!!!");
            }
        } else {
            turretMotor.setPower(0);
        }
    }

    public void operateOdomTracking(double currentX, double currentY, double chassisNormalizedHeading, boolean blueAlliance, boolean vinWantsToShoot) {
        // calculate TRUE target heading
        double currentAngle = getCurrentTurretAngle();
        double x_dist = (blueAlliance ? Globals.blueGoalX : Globals.redGoalX) - currentX;
        double y_dist = (blueAlliance ? Globals.blueGoalY : Globals.redGoalY) - currentY;
        double true_target_heading = Math.toDegrees(Math.atan2(y_dist, x_dist));
        turretTargetAngle = normalize(true_target_heading - chassisNormalizedHeading);

        targetInRange = turretTargetAngle > RIGHT_LIMIT && turretTargetAngle < LEFT_LIMIT;
        atTargetAngle = Math.abs(currentAngle - turretTargetAngle) < AT_TARGET_RANGE;

        if (targetInRange && vinWantsToShoot) { // in code-limited range
            output = update();
            if (currentAngle < LEFT_LIMIT && currentAngle > RIGHT_LIMIT) { // within hardstops, so good
                turretMotor.setPower(output);
            } else if (currentAngle >= LEFT_LIMIT && output < 0) { // past left limit, but positive output, so good
                turretMotor.setPower(output);
            } else if (currentAngle <= RIGHT_LIMIT && output > 0) { // past right limit, but negative output, so good
                turretMotor.setPower(output);
            } else { // must be both outside range && target is also outside range, so power 0
                turretMotor.setPower(0);
            }
        } else { // out of code limited range, or vincent doesn't wanna shoot, so default to center
            turretTargetAngle = 0;
            output = update();

            if (currentAngle < LEFT_LIMIT && currentAngle > RIGHT_LIMIT) {
                turretMotor.setPower(output);
            } else if (currentAngle >= LEFT_LIMIT && output < 0) {
                turretMotor.setPower(output);
            } else if (currentAngle <= RIGHT_LIMIT && output > 0) {
                turretMotor.setPower(output);
            } else {
                turretMotor.setPower(0);
            }
        }

//        opmode.telemetry.addData("true_target_heading", true_target_heading); // relative to field
        opmode.telemetry.addLine("\nTURRET");
        opmode.telemetry.addData("turretTargetAngle", turretTargetAngle); // between -180 and 180?
        opmode.telemetry.addData("turretCurrentAngle", currentAngle); // between -110 and 110
        opmode.telemetry.addData("target in turret range?", targetInRange);
//        opmode.telemetry.addData("turret output", output); // pos is clockwise
    }

    public void operateOdomAuto(double currentX, double currentY, double pedroHeadingDegrees, boolean blueAlliance, boolean runTurret) {
        // calculate TRUE target heading
        double currentAngle = getCurrentTurretAngle();
        double x_dist = (blueAlliance ? Globals.blueGoalX : Globals.redGoalX) - currentX;
        double y_dist = (blueAlliance ? Globals.blueGoalY : Globals.redGoalY) - currentY;
        double true_target_heading = Math.toDegrees(Math.atan2(y_dist, x_dist));
        turretTargetAngle = normalize(true_target_heading - normalize(pedroHeadingDegrees));

        targetInRange = turretTargetAngle > RIGHT_LIMIT && turretTargetAngle < LEFT_LIMIT;
        atTargetAngle = Math.abs(currentAngle - turretTargetAngle) < AT_TARGET_RANGE;

        if (targetInRange && runTurret) { // in code-limited range
            output = update();
            if (currentAngle < LEFT_LIMIT && currentAngle > RIGHT_LIMIT) { // within hardstops, so good
                turretMotor.setPower(output);
            } else if (currentAngle >= LEFT_LIMIT && output < 0) { // past left limit, but positive output, so good
                turretMotor.setPower(output);
            } else if (currentAngle <= RIGHT_LIMIT && output > 0) { // past right limit, but negative output, so good
                turretMotor.setPower(output);
            } else { // must be both outside range && target is also outside range, so power 0
                turretMotor.setPower(0);
            }
        } else { // out of code limited range, so default to center
            if (!targetInRange) {
                opmode.telemetry.addLine("OUT OF TURRET RANGE");
                opmode.telemetry.addLine("OUT OF TURRET RANGE");
                opmode.telemetry.addLine("OUT OF TURRET RANGE");
                opmode.telemetry.addLine("OUT OF TURRET RANGE");
            }
            turretTargetAngle = 0;
            output = update();

            if (currentAngle < LEFT_LIMIT && currentAngle > RIGHT_LIMIT) {
                turretMotor.setPower(output);
            } else if (currentAngle >= LEFT_LIMIT && output < 0) {
                turretMotor.setPower(output);
            } else if (currentAngle <= RIGHT_LIMIT && output > 0) {
                turretMotor.setPower(output);
            } else {
                turretMotor.setPower(0);
            }
        }

        opmode.telemetry.addLine("TURRET TELEMETRY");
        opmode.telemetry.addData("turretTargetAngle ", turretTargetAngle);
        opmode.telemetry.addData("turretCurrentAngle ", currentAngle);
        opmode.telemetry.addData("targetInRange?", targetInRange);
        opmode.telemetry.addLine("\n");
    }

    public double update() {
        // calculate the error
        double error = turretTargetAngle - getCurrentTurretAngle();

        double derivative = (error - lastError) / timer.seconds();

        double output = (Kp * error) + (Kd * derivative);
        output = Math.signum(output) * Math.sqrt(Math.min(1, Math.abs(output))); // TODO maybe remove sqrt PID?

        output = Math.max(-POWER_CLAMP, Math.min(POWER_CLAMP, output));

        // reset stuff for next update
        timer.reset();
        lastError = error;
        return output;
    }

    public double getCurrentTurretAngle() { // relative to bot
        return turretMotor.getCurrentPosition() * DEGREES_PER_TICK;
    }

    public void toggleSetPoses(boolean blueAlliance) {
        if (use110) {
            nonLLSetAngle = blueAlliance ? 110 : -110;
            use110 = false;
        } else {
            nonLLSetAngle = blueAlliance ? -71 : 71;
            use110 = true;
        }
    }

    private double normalize(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

}
