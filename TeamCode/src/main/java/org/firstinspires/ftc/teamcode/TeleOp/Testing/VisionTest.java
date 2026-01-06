package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.LimelightVision;
import org.firstinspires.ftc.teamcode.Subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.List;

@Configurable
@TeleOp(name = "LimeLight Test", group = "Testing")
public class VisionTest extends OpMode {

    private RobotHardware robotHardware = new RobotHardware();
    private ElapsedTime elapsedtime;
    private List<LynxModule> allHubs;

    private LimelightVision llVision = new LimelightVision();

//    private double targetAngle;

    private volatile boolean blueAlliance = true;

    @Override
    public void init() {
        robotHardware.initialize(this);

        // loop time stuff
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        //activate LL poling
        llVision.initialize(this, robotHardware);
        // bulk caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        if (gamepad1.b || gamepad2.b) {
            blueAlliance = false;
        } else if (gamepad1.x || gamepad2.x) {
            blueAlliance = true;
        }
        telemetry.addLine("B for RED | X for BLUE");
        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");

        llVision.trackPose(blueAlliance);
    }
}
