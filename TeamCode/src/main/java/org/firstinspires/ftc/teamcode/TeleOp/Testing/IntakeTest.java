package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.List;

@Configurable
@TeleOp(name = "Intake Tuning", group = "Testing")
public class IntakeTest extends OpMode{
    private Intake intake = new Intake();
    private RobotHardware robotHardware = new RobotHardware();
    private List<LynxModule> allHubs;

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
