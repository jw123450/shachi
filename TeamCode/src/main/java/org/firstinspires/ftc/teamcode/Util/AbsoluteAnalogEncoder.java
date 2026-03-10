package org.firstinspires.ftc.teamcode.Util;;

import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AbsoluteAnalogEncoder implements HardwareDevice {
    private OpMode opmode;
    private final double DEG_PER_VOLT = 360.0 / 3.3; /// TODO
    private final AnalogInput encoder;
    private ElapsedTime timer = new ElapsedTime();
    private double lastAngle;
    private double vel = 0; // deg/sec

    public AbsoluteAnalogEncoder(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        this.encoder = robotHardware.rightTurretAnalog;
    }

    public double getCurrentTurretAngle() {
        double newAngle = encoder.getVoltage() * DEG_PER_VOLT - 180.0;
        vel = (newAngle - lastAngle) / timer.seconds(); // deg/sec
        timer.reset();
        lastAngle = newAngle;
        return lastAngle;
    }


    @Override
    public void disable() {
        // "take no action" (encoder.close() call in SDK)
    }

    @Override
    public String getDeviceType() {
        return "Dawg why would you even call this method";
    }
}