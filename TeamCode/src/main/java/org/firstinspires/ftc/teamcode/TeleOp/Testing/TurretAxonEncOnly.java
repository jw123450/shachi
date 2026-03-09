package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@TeleOp(name = "Turret Enc Only", group = "Testing")
public class TurretAxonEncOnly extends OpMode{
    private RobotHardware robotHardware = new RobotHardware();
    private AbsoluteAnalogEncoder enc;

    @Override
    public void init() {
        robotHardware.initialize(this);
        enc = new AbsoluteAnalogEncoder(this, robotHardware);
    }

    @Override
    public void loop() {
        telemetry.addData("voltage", robotHardware.rightTurretAnalog.getVoltage());
        telemetry.addData("angle", enc.getCurrentTurretAngle()); // (-180, 180) theoretically
    }
}
