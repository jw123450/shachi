package org.firstinspires.ftc.teamcode.Subsystem;


//import static android.os.SystemClock.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Configurable
public class Intake {
    OpMode opmode;
    public DcMotorEx intakeMotor;
    public Servo ballPusher;
    private VoltageSensor battery;


    public enum IntakeState {
        INTAKING,
        INTAKING_FULL_POWER,
        REVERSE,
        IDLE
    }

    // Target current limit (amps)
    private static double CURRENT_LIMIT = 5.0;

    private static double STALL_WARNING_THRESHOLD = 5;

    // servo constants
    public static double BALL_PUSHER_STOW = 0.635;
    public static double BALL_PUSHER_DEPLOY = 0.4;
    public static double TUNING_INCREMENT = 0.001;

    // motor constants
    public static double INTAKING_FULL_POWER = -0.9;
    public static double INTAKING_POWER = -0.7;
    public static double REVERSE_POWER = 0.8;
    public static double IDLE_POWER = 0;

    // constantly updating states
    public volatile IntakeState intakeState = IntakeState.IDLE;
    public volatile boolean ballPusherDeployed = false;
    public volatile double targetPower = 0;
    public volatile boolean motorStalled = false;

    public Intake() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        intakeMotor = robotHardware.intakeMotor;
        ballPusher = robotHardware.ballPusher;
        battery = robotHardware.battery;
    }

    public void operateTesting() {
        if (Math.abs(opmode.gamepad1.left_stick_y) > 0.1) { intakeMotor.setPower(-opmode.gamepad1.left_stick_y); }

        // tuning servo poses
        if (opmode.gamepad1.a) { stowBallPusher(); }
        else if (opmode.gamepad1.b) { deployBallPusher(); }
        else if (opmode.gamepad1.dpad_up) { incremental(-1); }
        else if (opmode.gamepad1.dpad_down){ incremental(1); }

        opmode.telemetry.addData("intake amps pulled", intakeMotor.getCurrent(CurrentUnit.AMPS));
        opmode.telemetry.addData("motor power: ", intakeMotor.getPower());
        opmode.telemetry.addData("pusher servo pos: ", ballPusher.getPosition());
        opmode.telemetry.addData("pusher deployed? ", ballPusherDeployed);
        opmode.telemetry.addData("intake state enum: ", intakeState);
    }

    public void ioTeleOp() {
        if (opmode.gamepad1.right_trigger > 0.9 && this.intakeState != IntakeState.INTAKING_FULL_POWER) {
            intakeFullPower();
        } else if (opmode.gamepad1.right_trigger > 0.1 && this.intakeState != IntakeState.INTAKING){
            intake();
        } else if (opmode.gamepad1.a && this.intakeState != IntakeState.REVERSE) {
            reverse();
        } else if (this.intakeState != IntakeState.IDLE && this.intakeState != IntakeState.REVERSE){
            idle();
        }

//        opmode.telemetry.addData("intake state enum: ", intakeState);
//        opmode.telemetry.addData("intake motor power: ", intakeMotor.getPower());
//        opmode.telemetry.addData("pusher servo pos: ", ballPusher.getPosition());
    }

    public void kauaiTeleOp() {
        ioTeleOp();
    }

    public void operateCurrentLimiting() {
        double pulledCurrent = intakeMotor.getCurrent(CurrentUnit.AMPS);
        double assignedPower = limitPower(targetPower, pulledCurrent);
        intakeMotor.setPower(assignedPower);
        motorStalled = pulledCurrent > STALL_WARNING_THRESHOLD;

        opmode.telemetry.addLine("\nINTAKE");
        opmode.telemetry.addData("intake targetPower", targetPower);
        opmode.telemetry.addData("intake assignedPower", assignedPower);
        opmode.telemetry.addData("intake amps pulled", pulledCurrent);
//        opmode.telemetry.addData("motorStalled? ", motorStalled);
    }

//        if (intake.intakeState == Intake.IntakeState.IDLE) {
//            if (currentMagState != Spindexer.MagState.FULL) {
//                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
//                    // start intaking
//                    runningActions.add(new InstantAction(() -> intake.intake()));
//                }
//            }
//            else if (currentMagState == Spindexer.MagState.FULL) {
//                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
//                    // start intaking
//                    runningActions.add(new SequentialAction(
//                            new InstantAction(() -> intake.reverse()),
//                            new SleepAction(0.7),
//                            new InstantAction(() -> intake.idle())
//                    ));
//                }
//            }
//        } else if (intake.intakeState == Intake.IntakeState.INTAKING) {
//            if (currentMagState != Spindexer.MagState.FULL) {
//                // keep trying to intake unless driver presses button
//                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
//                    // flip up stop intaking
//                    runningActions.add(new InstantAction(() -> intake.idle()));
//                }
//            } else if (intake.chamberState != COLOR_TO_REJECT) {
//                // yay grabbed 3 balls, can stow now
//                runningActions.add(new SequentialAction(
//                        new InstantAction(() -> gamepad1.rumble(400)), // still massive delay?
//                        new InstantAction(() -> intake.reverse()),
//                        new SleepAction(0.7),
//                        new InstantAction(() -> intake.idle())
//                ));
//            }
//        } else if (intake.intakeState == Intake.IntakeState.REVERSE) {
//            // nothing, will never be in this state for an indefinite time
//        }


    // intake helper methods
    public void setIntake(double power) {
        targetPower = power;
    }
    public void intakeFullPower() {
        intakeState = IntakeState.INTAKING;
        setIntake(INTAKING_FULL_POWER);
    }
    public void intake() {
        intakeState = IntakeState.INTAKING;
        setIntake(INTAKING_POWER);
    }
    public void reverse() {
        intakeState = IntakeState.REVERSE;
        setIntake(REVERSE_POWER);
    }
    public void idle() {
        intakeState = IntakeState.IDLE;
        setIntake(IDLE_POWER);
    }

    // ball pusher helper methods
    public void deployBallPusher() {
        ballPusher.setPosition(BALL_PUSHER_DEPLOY);
        ballPusherDeployed = true;
    }

    public void stowBallPusher() {
        ballPusher.setPosition(BALL_PUSHER_STOW);
        ballPusherDeployed = false;
    }

    public void incremental(int sign) {
        ballPusher.setPosition(ballPusher.getPosition() + sign * TUNING_INCREMENT);
    }

    private double limitPower(double powerCmd, double pulledCurrent) {
        if (pulledCurrent > CURRENT_LIMIT) {
            double scale = CURRENT_LIMIT / pulledCurrent;
            powerCmd *= scale;
        }
        return powerCmd;
    }
}