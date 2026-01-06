package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo; //Gobilda PWM RGB light treated as a Servo

import java.util.HashMap;
import java.util.Timer;


public class RGBLights {
    public Servo RGBIndicator;
    private OpMode opmode;
    // map from name to corresponding servo position (0.0-1.0)
    private HashMap<Colors, Double> colors = new HashMap<>();
    private final double RAINBOW_CYCLE_TIME = 2.0; // seconds for full color cycle
    private final double RAINBOW_LOW_ENDPOINT = 0.29;
    private final double RAINBOW_HIGH_ENDPOINT = 0.722;

    public enum Colors {
        OFF,
        RED,
        ORANGE,
        YELLOW,
        SAGE,
        GREEN,
        AZURE,
        BLUE,
        INDIGO,
        VIOLET,
        WHITE
    }

    public volatile Colors currentColor = Colors.WHITE;

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.RGBIndicator = robotHardware.RGBIndicator;
        this.opmode = opmode;
        // Servo positions from the goBILDA chart https://www.gobilda.com/rgb-indicator-light-pwm-controlled/?srsltid=AfmBOopo3iYNvDaQDFttTQRF6RCs8uUW_0dftYO7PsB9ynAV060EuxIl
        colors.put(Colors.OFF, 0.0);
        colors.put(Colors.RED, 0.29);
        colors.put(Colors.ORANGE, 0.333);
        colors.put(Colors.YELLOW, 0.388);
        colors.put(Colors.SAGE, 0.444);
        colors.put(Colors.GREEN, 0.500);
        colors.put(Colors.AZURE, 0.555);
        colors.put(Colors.BLUE, 0.611);
        colors.put(Colors.INDIGO, 0.666);
        colors.put(Colors.VIOLET, 0.722);
        colors.put(Colors.WHITE, 1.0);

        setColor(Colors.WHITE);
    }

    public void setColor(Colors color) {
        currentColor = color;
        RGBIndicator.setPosition(colors.get(color));
    }

    public void operateAuto() {
        double t = opmode.getRuntime() / RAINBOW_CYCLE_TIME; // grows quite slowly

        // linear zigzag: 0 → 1 → 0 → 1 → 0
        double zigzag = Math.abs((t % 2) - 1);

        // map zigzag (0→1) to pose range (RAINBOW_LOW_ENDPOINT → RAINBOW_HIGH_ENDPOINT)
        double pos = RAINBOW_LOW_ENDPOINT + zigzag * (RAINBOW_HIGH_ENDPOINT - RAINBOW_LOW_ENDPOINT);
        RGBIndicator.setPosition(pos);
    }
}
