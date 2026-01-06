package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.RobotHardware;

public class BrakePad {
    OpMode opmode;
    Servo s;

    public static double TUNING_INCREMENT = 0.001;
    public static double DEPLOYED_POS = 0.335;
    public static double STOWED_POS = 0.46;

    public volatile boolean brakePadDeployed = false;

    public BrakePad() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        s = robotHardware.brakePadServo;
    }

    public void operateTuning() { /// if you need constant loop during tele or auto
        if (opmode.gamepad1.dpad_up) {
            incremental(1);
        } else if (opmode.gamepad1.dpad_down) {
            incremental(-1);
        }
        if (opmode.gamepad1.a) {
            deployBrake();
        } else if (opmode.gamepad1.b) {
            stowBrake();
        }

        opmode.telemetry.addData("pose", s.getPosition());
    }

    public void incremental(int sign) {
        s.setPosition(s.getPosition() + sign * TUNING_INCREMENT);
    }

    public void deployBrake() {
        s.setPosition(DEPLOYED_POS);
        brakePadDeployed = true;
    }

    public void stowBrake() {
        s.setPosition(STOWED_POS);
        brakePadDeployed = false;
    }
}
