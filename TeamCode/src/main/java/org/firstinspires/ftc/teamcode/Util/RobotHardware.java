package org.firstinspires.ftc.teamcode.Util;

//import com.acmerobotics.roadrunner.ftc.PinpointIMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {
    public DcMotorEx intakeMotor;
    public DcMotorEx Br, Bl, Fr, Fl;
    public DcMotorEx S1, S2;
    public DcMotorEx turretMotor;

    public Servo ballPusher;
    public Servo shooterLatch;
    public Servo RGBIndicator;
    public Servo brakePadServo;

    public VoltageSensor battery;

    public Limelight3A limelight;

    public GoBildaPinpointDriver odo;

    /// color + other sensors

    public void initialize(OpMode opmode) {
        /*
            CONFIG:

            pinpoint - CHUB i2c 1
            color sensor 0 - CHUB digital #
            color sensor 1 - CHUB digital #
            color sensor 2 - CHUB digital #

            MOTOR
            Bl - EHUB0
            Br - EHUB1
            Fl - EHUB2
            Fr - EHUB3
            Shooter1 (S1) - CHUB0
            Shooter2 (S2) - CHUB1
            turret - CHUB2
            intake - CHUB3

            SERVO
            ballPusher - EHUB0
            shooterLatch - CHUB0
         */
        battery = opmode.hardwareMap.voltageSensor.iterator().next();

        odo = opmode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        limelight = opmode.hardwareMap.get(Limelight3A.class, "limelight");
        opmode.telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        //intakeColorSensor = opmode.hardwareMap.get(RevColorSensorV3.class, "color");

        Fl = opmode.hardwareMap.get(DcMotorEx.class, "fl");
        Fr = opmode.hardwareMap.get(DcMotorEx.class, "fr");
        Bl = opmode.hardwareMap.get(DcMotorEx.class, "bl");
        Br = opmode.hardwareMap.get(DcMotorEx.class, "br");
        S1 = opmode.hardwareMap.get(DcMotorEx.class, "Shooter1");
        S2 = opmode.hardwareMap.get(DcMotorEx.class, "Shooter2");
        turretMotor = opmode.hardwareMap.get(DcMotorEx.class, "Turret");
        intakeMotor = opmode.hardwareMap.get(DcMotorEx.class, "intake");
        ballPusher = opmode.hardwareMap.get(Servo.class, "intakeservo");
        shooterLatch = opmode.hardwareMap.get(Servo.class, "shooterlatch");
        RGBIndicator = opmode.hardwareMap.get(Servo.class, "light");
        brakePadServo = opmode.hardwareMap.get(Servo.class, "brake");

        Fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        S2.setDirection(DcMotorSimple.Direction.REVERSE);
        S1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        S2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Fl.setDirection(DcMotorSimple.Direction.REVERSE);
        Bl.setDirection(DcMotorSimple.Direction.REVERSE);
        Br.setDirection(DcMotorSimple.Direction.FORWARD);
        Fr.setDirection(DcMotorSimple.Direction.FORWARD);

        odo.setOffsets(108.647, 40.032, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }
}
