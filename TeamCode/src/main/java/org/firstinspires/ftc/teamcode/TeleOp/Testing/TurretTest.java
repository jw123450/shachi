package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.LimelightVision;
import org.firstinspires.ftc.teamcode.Subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.Turret;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.List;

@TeleOp(name = "Turret Test + Tuning", group = "Testing")
public class TurretTest extends OpMode {

    private MecanumDrive drive = new MecanumDrive(); // TODO: motor + pinpoint config
    private RobotHardware robotHardware = new RobotHardware();
    private Turret turret = new Turret();
    private LimelightVision llVision = new LimelightVision();

    private ElapsedTime elapsedtime;
    private List<LynxModule> allHubs;
    FtcDashboard dash = FtcDashboard.getInstance();
    private MultipleTelemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

    private volatile boolean blueAlliance = true;

    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        robotHardware.initialize(this);
        llVision.initialize(this, robotHardware);
        drive.initialize(this, robotHardware);
        turret.initialize(this, robotHardware, llVision, false);

        // loop time stuff
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        //activate LL poling
        // bulk caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // loop time stuff
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
    }

    @Override
    public void init_loop() {
        if (gamepad1.b || gamepad2.b) {
            blueAlliance = false;
        } else if (gamepad1.x || gamepad2.x) {
            blueAlliance = true;
        }
        telemetry.addLine("B for RED | X for BLUE");
        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
        telemetry.update();
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) { hub.clearBulkCache(); }
        TelemetryPacket packet = new TelemetryPacket();
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.b && !previousGamepad1.b) {
            blueAlliance = !blueAlliance;
        }

        llVision.trackPose(blueAlliance);
        turret.operateTesting(packet, blueAlliance);
        drive.operateSimple();

        // +69 and -110 set poses
        if (currentGamepad1.x &&!previousGamepad1.x) {
            turret.useLL = false;
            turret.toggleSetPoses(blueAlliance);
        } else if (currentGamepad1.y && !previousGamepad1.y) {
            turret.useLL = true;
        }

        dash.sendTelemetryPacket(packet);
        telemetry.addLine("B for RED | X for BLUE");
        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();

    }
}
