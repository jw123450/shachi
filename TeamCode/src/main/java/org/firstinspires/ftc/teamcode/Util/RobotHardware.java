package org.firstinspires.ftc.teamcode.Util;

//import com.acmerobotics.roadrunner.ftc.PinpointIMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {
    public DcMotorEx intakeMotor, transferMotor;
    public DcMotorEx Br, Bl, Fr, Fl;
    public DcMotorEx ShooterR, ShooterL;

    public Servo leftTurretServo, rightTurretServo;
    public Servo leftIntakeServo, rightIntakeServo;
    public Servo RGBIndicatorL, RGBIndicatorR;
    public Servo hoodAngleAdjust, shooterLatch;

    public AnalogInput rightTurretAnalog;
    public DigitalChannel transferBreakBeam, intakeBreakBeam;

    public VoltageSensor battery;

    public Limelight3A limelight;

    public GoBildaPinpointDriver odo;

    /// color + other sensors

    public void initialize(OpMode opmode) {
        /*
            CONFIG:

            SENSORS
            pinpoint            - CHUB i2c 1
            turret analog input - CHUB analog 3
            intake dist sens   - CHUB digital 0
            transfer dist sens - CHUB digital 4

            MOTOR
            Fl - EHUB0
            Fr - EHUB1
            Bl - EHUB2
            Br - EHUB3
            ShooterL - CHUB0
            ShooterR - CHUB1
            transfer - CHUB2
            intake   - CHUB3

            SERVO
            leftTurretServo  - SHUB 3
            rightTurretServo - SHUB 2
            Intake Servo L   - EHUB 0
            RGB light L      - EHUB 5
            Intake Servo R   - CHUB 5
            shooterLatch     - CHUB 1
            hoodAngleAdjust  - CHUB 0
            RGB light R      - CHUB 2

         */
        battery = opmode.hardwareMap.voltageSensor.iterator().next();

        odo = opmode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        intakeBreakBeam = opmode.hardwareMap.get(DigitalChannel.class, "intakeBreak");
        transferBreakBeam = opmode.hardwareMap.get(DigitalChannel.class, "transferBreak");
        intakeBreakBeam.setMode(DigitalChannel.Mode.INPUT);
        transferBreakBeam.setMode(DigitalChannel.Mode.INPUT);
        /// telemetry.addData("intake beam state", intakeBreakBeam.getState())

        rightTurretAnalog = opmode.hardwareMap.get(AnalogInput.class, "turretAnalog");

        limelight = opmode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

//        opmode.telemetry.setMsTransmissionInterval(11);

        Fl = opmode.hardwareMap.get(DcMotorEx.class, "fl");
        Fr = opmode.hardwareMap.get(DcMotorEx.class, "fr");
        Bl = opmode.hardwareMap.get(DcMotorEx.class, "bl");
        Br = opmode.hardwareMap.get(DcMotorEx.class, "br");
        ShooterL = opmode.hardwareMap.get(DcMotorEx.class, "shooterL");
        ShooterR = opmode.hardwareMap.get(DcMotorEx.class, "shooterR");
        transferMotor = opmode.hardwareMap.get(DcMotorEx.class, "transfer");
        intakeMotor = opmode.hardwareMap.get(DcMotorEx.class, "intake");

        leftTurretServo = opmode.hardwareMap.get(Servo.class, "turretL");
        rightTurretServo = opmode.hardwareMap.get(Servo.class, "turretR");
        leftIntakeServo = opmode.hardwareMap.get(Servo.class, "intakeL");
        rightIntakeServo = opmode.hardwareMap.get(Servo.class, "intakeR");
        hoodAngleAdjust = opmode.hardwareMap.get(Servo.class, "hoodangle");
        shooterLatch = opmode.hardwareMap.get(Servo.class, "latch");

        RGBIndicatorL = opmode.hardwareMap.get(Servo.class, "lightL");
        RGBIndicatorR = opmode.hardwareMap.get(Servo.class, "lightR");

        Fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        ShooterR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        ShooterL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        Fl.setDirection(DcMotorSimple.Direction.REVERSE);
        Bl.setDirection(DcMotorSimple.Direction.REVERSE);
        Br.setDirection(DcMotorSimple.Direction.FORWARD);
        Fr.setDirection(DcMotorSimple.Direction.FORWARD);

        // xOffset is same as pedro ForwardPodY
        // yOffest is same as pedro StrafePodX
        odo.setOffsets(-68.538, -132.605, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }
}
