package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.RobotHardware;


@Configurable
public class SubsystemTemplate {
    OpMode opmode;
    // TODO: add all motors, servos, sensors
    DcMotorEx m;
    Servo s;

    // TODO: add necessary constants
    public static double constant = 0;

    public SubsystemTemplate() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        // TODO: connect local objects to pre-initialized objects (example below)
        // m = robotHardware.asdfajsda;
    }

    public void operateTesting() { /// if you need constant loop during tele or auto

        // example debugging telemetry: opmode.telemetry.addData("debugging messages", s.getPosition());
    }

    public void operateTeleOp() { /// if you need constant loop during tele or auto

    }

    public void helper() {
        // abstract things like setting motor power, servo position, etc.
    }
}
