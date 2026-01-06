package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.Turret;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@TeleOp(name = "Reset Turret Encoder", group = "B")
public class ResetTurretEncoder extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    Turret turret = new Turret();

    @Override
    public void init() {
        robotHardware.initialize(this);
        turret.initialize(this, robotHardware, null, true);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

    }
}
