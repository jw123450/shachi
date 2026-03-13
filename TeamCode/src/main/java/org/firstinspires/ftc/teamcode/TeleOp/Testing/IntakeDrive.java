package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.List;

@Configurable
@TeleOp(name = "Intake + Drive Only", group = "Testing")
public class IntakeDrive extends OpMode{
    private Intake intake = new Intake();
    private MecanumDrive drive = new MecanumDrive();
    private RobotHardware robotHardware = new RobotHardware();

    @Override
    public void init() {
        robotHardware.initialize(this);
        intake.initialize(this, robotHardware);
        drive.initialize(this, robotHardware);
    }

    @Override
    public void loop() {
        drive.operateSimple();
        intake.operateSimple(true);
        // dpad_up: incremental -
        // dpad_down: incremental +
        // A: stow (after changing values and loading onto bot)
        // B: deploy (after changing values)

        // POWER TESTING
        // joystick: variable power
    }
}
