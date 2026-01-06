package org.firstinspires.ftc.teamcode.TeleOp;

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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.SubsystemTemplate;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.List;

@TeleOp(name = "______ Test", group = "Testing") // TODO
public class TestTeleOpTemplate extends OpMode{

    /// for testing single subsystems at a time

    private SubsystemTemplate template = new SubsystemTemplate(); // TODO
    RobotHardware robotHardware = new RobotHardware();
    private ElapsedTime elapsedtime;
    private List<LynxModule> allHubs;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        robotHardware.initialize(this);
        template.initialize(this, robotHardware);

        // loop time stuff
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        // panels display
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // bulk caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) { hub.clearBulkCache(); }

        //  rising edge detection
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // ADD INSTRUCTIONS
        template.operateTesting();

    // loop time measuring
//        telemetry.addData("Loop Times", elapsedtime.milliseconds());
//        elapsedtime.reset();

        telemetryM.debug("variable: " + "variable.getValue()");
        telemetryM.update(telemetry);
    }
}