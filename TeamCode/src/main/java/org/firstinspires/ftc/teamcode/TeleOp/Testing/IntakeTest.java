package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.List;

@Configurable
@TeleOp(name = "Intake Tuning", group = "Testing")
public class IntakeTest extends OpMode{
    private Intake intake = new Intake();
    private RobotHardware robotHardware = new RobotHardware();

    @Override
    public void init() {
        robotHardware.initialize(this);
        intake.initialize(this, robotHardware);
    }

    @Override
    public void loop() {
        intake.operateTesting();
        // dpad_up: incremental -
        // dpad_down: incremental +
        // A: stow (after changing values and loading onto bot)
        // B: deploy (after changing values)

        // POWER TESTING
        // joystick: variable power
    }
}
