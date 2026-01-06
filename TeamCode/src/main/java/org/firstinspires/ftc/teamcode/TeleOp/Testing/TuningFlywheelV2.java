package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.ShooterV2;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.List;

@TeleOp(name = "Counter-roller Flywheel Tuning", group = "Testing")
public class TuningFlywheelV2 extends OpMode{

    private ShooterV2 shooter = new ShooterV2();
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
        shooter.initialize(this, robotHardware);

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

        if (gamepad1.left_bumper) {
            /// Manual power control with GP1 left joystick Y
            shooter.operateFindMaxRPM();
        } else {
            /// TUNING FLYWHEEL PID
            /// HOLD left trigger to activate PID
            /// REFERENCE GOOGLE DOC FOR MORE

            /// TUNING SERVO LATCH (read driver hub telemetry or FTC dash)
            // Y: incremental +
            // A: incremental -

            /// TUNING HOOD ADJUST
            // B: incremental +
            // X: incremental -

            shooter.operateTuning(packet);
        }

        /// if we want to test realistic rapid fire
        robotHardware.intakeMotor.setPower(-gamepad1.right_trigger);

        // loop time measuring
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();

        dash.sendTelemetryPacket(packet);
    }
}