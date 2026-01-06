package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.FlywheelShooter;
import org.firstinspires.ftc.teamcode.Subsystem.SubsystemTemplate;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.List;

//@Config
@TeleOp(name = "Flywheel Tuning", group = "Testing")
public class TuningFlywheel extends OpMode{

    private FlywheelShooter shooter = new FlywheelShooter();
    RobotHardware robotHardware = new RobotHardware();
    private ElapsedTime elapsedtime;
    private List<LynxModule> allHubs;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private MultipleTelemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        robotHardware.initialize(this);
        shooter.initialize(this, robotHardware, null);

        // loop time stuff
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        // bulk caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) { hub.clearBulkCache(); }

        TelemetryPacket packet = new TelemetryPacket();

        //  rising edge detection
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (gamepad1.right_bumper) {
            shooter.operateFindMaxRPM(); // already done?
        } else {
            // TODO (delete when done)
            /// TUNING FLYWHEEL PID INSTRUCTIONS
            /// Connect to robot WiFi, open FTC dashboard at http://192.168.43.1:8080/dash
            /// HOLD left trigger to activate PID
            /// vary targetRPM with dpad_up (far), dpad_down (near), or dpad_left (0)
            /// looking at graph, ADJUST kV FIRST, then kP and kD
            /// put balls through and shoot them to see how well it recovers (look at graph)

            // TODO (delete when done)
            /// TUNING SERVO LATCH (read driver hub telemetry or FTC dash)
            // Y: incremental +
            // A: incremental -
            // B: open latch (after updating LATCH_OPEN_POS and pushing onto bot)
            // X: close latch (after updating and pushing)

            shooter.operateTuning(packet); //0 for latch
        }

        // loop time measuring
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();

        dash.sendTelemetryPacket(packet);
    }
}