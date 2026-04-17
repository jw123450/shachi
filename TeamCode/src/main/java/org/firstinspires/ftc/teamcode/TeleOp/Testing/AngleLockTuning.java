package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.Util.PinpointManager;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Configurable
@TeleOp(name = "Angle Lock Tuning", group = "Testing")
public class AngleLockTuning extends OpMode{
    private Intake intake = new Intake();
    private MecanumDrive drive = new MecanumDrive();
    private RobotHardware robotHardware = new RobotHardware();
    private PinpointManager pinpoint = new PinpointManager();
    private volatile boolean blueAlliance = true;

    @Override
    public void init() {
        robotHardware.initialize(this);
        intake.initialize(this, robotHardware);
        drive.initialize(this, robotHardware);
        pinpoint.initialize(this, robotHardware);
    }

    @Override
    public void loop() {
        if (gamepad1.b) {
            blueAlliance = false;
        } else if (gamepad1.x) {
            blueAlliance = true;
        }
        drive.operateTeleOp(pinpoint.normalizedHeading, blueAlliance);
        intake.operateTeleOp(true, false);
        pinpoint.operateTrackingPose();

        telemetry.addLine("\nPOSE");
        telemetry.addData("pp X", pinpoint.X);
        telemetry.addData("pp Y", pinpoint.Y);
        telemetry.addData("pp heading (deg)", pinpoint.normalizedHeading);
    }
}
