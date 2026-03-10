package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Configurable
public class Intake {
    OpMode opmode;
    public DcMotorEx intakeMotor, transferMotor;
    public Servo leftIntakeArm, rightIntakeArm;


    public enum IntakeState {
        INTAKING,
        INTAKING_FULL_POWER,
        REVERSE,
        IDLE,
        TRANSFERRING
    }

    // servo constants
    public static double INTAKE_STOW = 0.023;
    public static double INTAKE_DEPLOY = 0.32;
    public static double TUNING_INCREMENT = 0.001;

    // motor constants
    public static double INTAKING_FULL_POWER = 0.9;
    public static double INTAKING_POWER = 0.7;
    public static double REVERSE_POWER = -0.8;
    public static double IDLE_POWER = 0;

    // constantly updating states
    public volatile IntakeState intakeState = IntakeState.IDLE;
    public boolean isArmStowed = true;
    public boolean isTransferring = false;
    public volatile double targetIntakePower = 0;
    public volatile double targetTransferPower = 0;
//    public volatile boolean motorStalled = false;

    public Intake() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        intakeMotor = robotHardware.intakeMotor;
        transferMotor = robotHardware.transferMotor;
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); /// add BRAKE MODE?

        leftIntakeArm = robotHardware.leftIntakeServo;
        rightIntakeArm = robotHardware.rightIntakeServo;
        rightIntakeArm.setDirection(Servo.Direction.REVERSE); /// maybe left reverse
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
    }

    public void operateSimple() {
        if (opmode.gamepad1.rightBumperWasPressed()) { // prob breaks if spam click, but fix pretty simple
            deployIntake();
        } else if (opmode.gamepad1.rightBumperWasReleased()) {
            stowIntake();
        }
        intakeMotor.setPower(opmode.gamepad1.right_trigger);
        transferMotor.setPower(opmode.gamepad1.left_trigger);
    }

    public void operateTeleOp() {
        intakeMotor.setPower(targetIntakePower);
        transferMotor.setPower(targetTransferPower);
    }

    ///  asfadsfsdafadsfasdfasaxfaasdf
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
        targetIntakePower = power;
    }
    public void setTransfer(double power) {
        targetTransferPower = power;
    }
    public void intakeFullPower() {
        intakeState = IntakeState.INTAKING;
        setIntake(INTAKING_FULL_POWER);
        setTransfer(INTAKING_FULL_POWER);
    }
    public void intake() {
        intakeState = IntakeState.INTAKING;
        setIntake(INTAKING_POWER);
        setTransfer(INTAKING_POWER);
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
        intakeState = IntakeState.TRANSFERRING;
        setIntake(IDLE_POWER);
        setTransfer(INTAKING_FULL_POWER);
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