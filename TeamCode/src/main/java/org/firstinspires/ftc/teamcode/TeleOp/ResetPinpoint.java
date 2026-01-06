package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.Turret;
import org.firstinspires.ftc.teamcode.Util.PinpointManager;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@TeleOp(name = "Reset Pinpoint", group = "B")
public class ResetPinpoint extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    PinpointManager pinpoint = new PinpointManager();

    @Override
    public void init() {
        robotHardware.initialize(this);
        pinpoint.initialize(this, robotHardware);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

    }
}
