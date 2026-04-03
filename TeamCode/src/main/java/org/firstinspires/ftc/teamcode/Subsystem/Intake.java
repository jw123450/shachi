package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Config
public class Intake {
    OpMode opmode;
    public DcMotorEx intakeMotor, transferMotor;
    public Servo leftIntakeArm, rightIntakeArm;
    public DigitalChannel transferBreakBeam, intakeBreakBeam;
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime transferTimer = new ElapsedTime();


    public enum IntakeState {
        INTAKING,
        REVERSE,
        IDLE
    }

    // servo constants
    public static double INTAKE_STOW = 0.023;
    public static double INTAKE_DEPLOY = 0.32;
    public static double TUNING_INCREMENT = 0.001;

    // motor constants
    public static double INTAKING_POWER = 0.9;
    public static double INTAKING_TRANSFER_POWER = 0.65;
    public static double REVERSE_POWER = -0.7;
    public static double IDLE_POWER = 0;

    public static double INTAKE_DEBOUNCE_DELAY = 200; // ms
    public static double TRANSFER_DEBOUNCE_DELAY = 80;

    // constantly updating states
    public volatile IntakeState intakeState = IntakeState.IDLE;
    public boolean isArmStowed = true;
    public boolean isFull, transferFull, intakeFull = true;
    public volatile double targetIntakePower = 0;
    public volatile double targetTransferPower = 0;

    public Intake() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        intakeMotor = robotHardware.intakeMotor;
        transferMotor = robotHardware.transferMotor;
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntakeArm = robotHardware.leftIntakeServo;
        rightIntakeArm = robotHardware.rightIntakeServo;
        rightIntakeArm.setDirection(Servo.Direction.REVERSE);

        transferBreakBeam = robotHardware.transferBreakBeam;
        intakeBreakBeam = robotHardware.intakeBreakBeam;
    }

    public void operateTesting() {
        if (Math.abs(opmode.gamepad1.left_stick_y) > 0.1) {
            intakeMotor.setPower(-opmode.gamepad1.left_stick_y);
        }
        if (Math.abs(opmode.gamepad1.right_stick_y) > 0.1) {
            transferMotor.setPower(-opmode.gamepad1.right_stick_y);
        }
        if (opmode.gamepad1.right_trigger > 0.1) {
            intakeMotor.setPower(opmode.gamepad1.right_trigger);
            transferMotor.setPower(opmode.gamepad1.right_trigger);
        } else if (Math.abs(opmode.gamepad1.left_stick_y) < 0.1 && Math.abs(opmode.gamepad1.right_stick_y) < 0.1) {
            intakeMotor.setPower(0);
            transferMotor.setPower(0);
        }

        // tuning servo poses
        if (opmode.gamepad1.y) { stowIntake(); }
        else if (opmode.gamepad1.a) { deployIntake(); }
        else if (opmode.gamepad1.dpad_up) { incremental(-1); }
        else if (opmode.gamepad1.dpad_down){ incremental(1); }

        opmode.telemetry.addData("intake amps ", intakeMotor.getCurrent(CurrentUnit.AMPS));
        opmode.telemetry.addData("transfer amps", transferMotor.getCurrent(CurrentUnit.AMPS));
        opmode.telemetry.addData("left servo pos: ", leftIntakeArm.getPosition());
        opmode.telemetry.addData("right servo pos: ", rightIntakeArm.getPosition());
        opmode.telemetry.addData("stowed? ", isArmStowed);
        opmode.telemetry.addData("intake state enum: ", intakeState);
        opmode.telemetry.addData("intake sens state ", intakeBreakBeam.getState());
        opmode.telemetry.addData("transfer sens state: ", transferBreakBeam.getState());

    }

    public void operateSimple(boolean separateMotors) {
        if (opmode.gamepad1.rightBumperWasPressed()) { // prob breaks if spam click, but fix pretty simple
            deployIntake();
        } else if (opmode.gamepad1.rightBumperWasReleased()) {
            stowIntake();
        }
        if (separateMotors) {
            intakeMotor.setPower(opmode.gamepad1.right_trigger);
            transferMotor.setPower(opmode.gamepad1.left_trigger);
        } else {
            intakeMotor.setPower(opmode.gamepad1.right_trigger);
            transferMotor.setPower(opmode.gamepad1.right_trigger);
        }
    }

    public void operateTeleOp(boolean useManualIntake) {
        /// breakbeams
        boolean intakeReading = intakeBreakBeam.getState();
        boolean transferReading = transferBreakBeam.getState();

        // state changed -> start debounce timer
        if (intakeReading != intakeFull) {
            if (intakeTimer.milliseconds() > INTAKE_DEBOUNCE_DELAY) {
                intakeFull = intakeReading;
                intakeTimer.reset();
            }
        } else {
            // reset timer if sensors match confirmed state
            intakeTimer.reset();
        }

        if (transferReading != transferFull) {
            if (transferTimer.milliseconds() > TRANSFER_DEBOUNCE_DELAY) {
                transferFull = transferReading;
                transferTimer.reset();
            }
        } else {
            // reset timer if sensors match confirmed state
            transferTimer.reset();
        }

        isFull = intakeFull && transferFull;

        /// intake + transfer logic
        if (useManualIntake) {
            if (opmode.gamepad1.right_trigger > 0.2) {
                if (transferFull) { runIntakeOnly(); }
                else { intakingIntake(); }
            } else if (opmode.gamepad1.aWasPressed() && intakeState != IntakeState.REVERSE) {
                reverse();
            } else if (opmode.gamepad1.aWasReleased() && intakeState == IntakeState.REVERSE) {
                idle();
            } else if (opmode.gamepad1.right_trigger <= 0.1 && intakeState != IntakeState.IDLE && intakeState != IntakeState.REVERSE) {
                idle();
            }
        }

        /// intake arm
        if (isFull && !isArmStowed) {
            stowIntake();
        } else if (opmode.gamepad1.rightBumperWasPressed() && !isFull) {
            deployIntake();
        } else if (opmode.gamepad1.rightBumperWasReleased() && !isFull) {
            stowIntake();
        }

        intakeMotor.setPower(targetIntakePower);
        transferMotor.setPower(targetTransferPower);

//        opmode.telemetry.addLine("\nIntake");
//        opmode.telemetry.addData("intake timer ms", intakeTimer.milliseconds());
//        opmode.telemetry.addData("transfer timer ms", transferTimer.milliseconds());
//        opmode.telemetry.addData("intakeReading", intakeReading);
//        opmode.telemetry.addData("transferReading", transferReading);
//        opmode.telemetry.addData("intakeFull", intakeFull);
//        opmode.telemetry.addData("transferFull", transferFull);
//        opmode.telemetry.addData("isFull", isFull);
    }

    public void operateAuto() {
        /// breakbeams
        boolean intakeReading = intakeBreakBeam.getState();
        boolean transferReading = transferBreakBeam.getState();

        // state changed -> start debounce timer
        if (intakeReading != intakeFull) {
            if (intakeTimer.milliseconds() > INTAKE_DEBOUNCE_DELAY) {
                intakeFull = intakeReading;
                intakeTimer.reset();
            }
        } else {
            // reset timer if sensors match confirmed state
            intakeTimer.reset();
        }

        if (transferReading != transferFull) {
            if (transferTimer.milliseconds() > TRANSFER_DEBOUNCE_DELAY) {
                transferFull = transferReading;
                transferTimer.reset();
            }
        } else {
            // reset timer if sensors match confirmed state
            transferTimer.reset();
        }

        isFull = intakeFull && transferFull;

        intakeMotor.setPower(targetIntakePower);
        transferMotor.setPower(targetTransferPower);

//        opmode.telemetry.addLine("\nIntake");
//        opmode.telemetry.addData("intake timer ms", intakeTimer.milliseconds());
//        opmode.telemetry.addData("transfer timer ms", transferTimer.milliseconds());
//        opmode.telemetry.addData("intakeReading", intakeReading);
//        opmode.telemetry.addData("transferReading", transferReading);
//        opmode.telemetry.addData("intakeFull", intakeFull);
//        opmode.telemetry.addData("transferFull", transferFull);
//        opmode.telemetry.addData("isFull", isFull);
    }



    // intake helper methods
    public void setIntake(double power) {
        targetIntakePower = power;
    }
    public void setTransfer(double power) {
        targetTransferPower = power;
    }
    public void shootingIntake() {
        intakeState = IntakeState.INTAKING;
        setIntake(INTAKING_POWER);
        setTransfer(INTAKING_POWER);
    }
    public void intakingIntake() {
        intakeState = IntakeState.INTAKING;
        setIntake(INTAKING_POWER);
        setTransfer(INTAKING_TRANSFER_POWER);
    }
    public void reverse() {
        intakeState = IntakeState.REVERSE;
        setIntake(REVERSE_POWER);
        setTransfer(REVERSE_POWER);
    }
    public void idle() {
        intakeState = IntakeState.IDLE;
        setIntake(IDLE_POWER);
        setTransfer(IDLE_POWER);
    }
    public void runTransferOnly() {
        intakeState = IntakeState.INTAKING;
        setIntake(IDLE_POWER);
        setTransfer(INTAKING_POWER);
    }

    public void runIntakeOnly() {
        intakeState = IntakeState.INTAKING;
        setIntake(INTAKING_POWER);
        setTransfer(IDLE_POWER);
    }

    // ball pusher helper methods
    public void deployIntake() {
        leftIntakeArm.setPosition(INTAKE_DEPLOY);
        rightIntakeArm.setPosition(INTAKE_DEPLOY);
        isArmStowed = false;
    }

    public void stowIntake() {
        leftIntakeArm.setPosition(INTAKE_STOW);
        rightIntakeArm.setPosition(INTAKE_STOW);
        isArmStowed = true;
    }

    public void toggleDropdown() {
        if (isArmStowed) {
            deployIntake();
        } else {
            stowIntake();
        }
    }

    public void incremental(int sign) {
        leftIntakeArm.setPosition(leftIntakeArm.getPosition() + sign * TUNING_INCREMENT);
        rightIntakeArm.setPosition(rightIntakeArm.getPosition() + sign * TUNING_INCREMENT);
    }

//    private double limitPower(double powerCmd, double pulledCurrent) {
//        if (pulledCurrent > CURRENT_LIMIT) {
//            double scale = CURRENT_LIMIT / pulledCurrent;
//            powerCmd *= scale;
//        }
//        return powerCmd;
//    }
}