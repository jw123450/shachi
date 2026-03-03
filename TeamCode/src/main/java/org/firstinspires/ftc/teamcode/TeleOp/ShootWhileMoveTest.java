//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.math.Vector;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Subsystem.Intake;
//import org.firstinspires.ftc.teamcode.Util.LimelightVision;
//import org.firstinspires.ftc.teamcode.Subsystem.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
//import org.firstinspires.ftc.teamcode.Subsystem.Turret;
//import org.firstinspires.ftc.teamcode.Util.Globals;
//import org.firstinspires.ftc.teamcode.Util.PinpointManager;
//import org.firstinspires.ftc.teamcode.Util.RGBLights;
//import org.firstinspires.ftc.teamcode.Util.RobotHardware;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@TeleOp(name = "Shoot While Move Test", group = "B")
//public class ShootWhileMoveTest extends OpMode {
//
//    private ElapsedTime elapsedtime;
//    private List<LynxModule> allHubs;
//    private FtcDashboard dash = FtcDashboard.getInstance();
//    private List<Action> runningActions = new ArrayList<>();
//
//    private RobotHardware robotHardware = new RobotHardware();
//    private Intake intake = new Intake();
//    private Turret turret = new Turret();
//    private MecanumDrive drive = new MecanumDrive();
//    private LimelightVision llVision = new LimelightVision();
//    private Shooter shooter = new Shooter();
//    private PinpointManager pinpoint = new PinpointManager();
//    private RGBLights lights = new RGBLights();
//
//    private final double MINIMUM_RPM = 2000;
//    private final double PUSHER_DELAY = 0.5;
//    private final double RAPID_FIRE_DELAY = 0.63;
//    private final double RGB_ALERT_DELAY = 1.5; // seconds
//
//    final Gamepad currentGamepad1 = new Gamepad();
//    final Gamepad currentGamepad2 = new Gamepad();
//    final Gamepad previousGamepad1 = new Gamepad();
//    final Gamepad previousGamepad2 = new Gamepad();
//
//    private volatile boolean blueAlliance = true;
//    private volatile boolean useManualIntake = true;
//    private volatile boolean vinWantsToShoot = false;
//    private volatile boolean cyclingFarZone = false;
//    private volatile boolean singleShot = false;
//
//    @Override
//    public void init() {
//        robotHardware.initialize(this);
//        pinpoint.initialize(this, robotHardware);
//        intake.initialize(this, robotHardware);
//        drive.initialize(this, robotHardware);
//        llVision.initialize(this, robotHardware);
//        shooter.initialize(this, robotHardware);
//        turret.initialize(this, robotHardware, false);
//        lights.initialize(this, robotHardware);
//
//        // loop time stuff
//        elapsedtime = new ElapsedTime();
//        elapsedtime.reset();
//
//        // Pull info from globals
//        pinpoint.transferAutoPose(Globals.autoEndPose);
//        blueAlliance = Globals.blueAlliance;
//        lights.setColor(blueAlliance ? RGBLights.Colors.BLUE : RGBLights.Colors.RED);
//
//        // bulk caching
//        allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
//        lights.setColor(blueAlliance ? RGBLights.Colors.BLUE : RGBLights.Colors.RED);
//    }
//
//    @Override
//    public void init_loop() {
//        if (gamepad1.b || gamepad2.b) {
//            blueAlliance = false;
//            lights.setColor(RGBLights.Colors.RED);
//        } else if (gamepad1.x || gamepad2.x) {
//            blueAlliance = true;
//            lights.setColor(RGBLights.Colors.BLUE);
//        }
//
//        pinpoint.operateTrackingPose();
//
//        telemetry.addLine("B for RED (either gamepad)");
//        telemetry.addLine("X for BLUE");
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
//
//        telemetry.addData("end pose X", Globals.autoEndPose.getX());
//        telemetry.addData("end pose Y", Globals.autoEndPose.getY());
//        telemetry.addData("end pose heading", Math.toDegrees(Globals.autoEndPose.getHeading()));
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        intake.stowBallPusher();
//        shooter.closeLatch();
//        pinpoint.transferAutoPose(Globals.autoEndPose);
//        shooter.autoRPMmode = true;
//        lights.setColor(RGBLights.Colors.WHITE);
//    }
//
//    @Override
//    public void loop() {
//        for (LynxModule hub : allHubs) { hub.clearBulkCache();}
//        previousGamepad1.copy(currentGamepad1);
//        previousGamepad2.copy(currentGamepad2);
//        currentGamepad1.copy(gamepad1);
//        currentGamepad2.copy(gamepad2);
//        // for RR Action execution
//        TelemetryPacket packet = new TelemetryPacket();
//        List<Action> newActions = new ArrayList<>();
//        for (Action action : runningActions) { if (action.run(packet)) { newActions.add(action); } }
//        runningActions = newActions;
//
//        /// check google slides for controls
//        // ALLIANCE SWITCH
//        if (gamepad2.b) {
//            blueAlliance = false;
//            alertAction(RGBLights.Colors.RED);
//        } else if (gamepad2.x) {
//            blueAlliance = true;
//            alertAction(RGBLights.Colors.BLUE);
//        }
//
//        // operate loops
//        pinpoint.operateTrackingPose(); // Changes X and Y to pedro coordinates
//        drive.operateSimple();
//
//        // toggling idle RPM
//        if (currentGamepad1.x && !previousGamepad1.x) {
//            if (cyclingFarZone) { // toggle from far to near
//                cyclingFarZone = false;
//                alertAction(RGBLights.Colors.YELLOW);
//            } else { // toggle from near to far
//                cyclingFarZone = true;
//                alertAction(RGBLights.Colors.VIOLET);
//            }
//        }
//
//        // default to orange or violet when no active alerts
//        if (lights.currentColor == RGBLights.Colors.WHITE) {
//            runningActions.add(new InstantAction(() -> lights.setColor(cyclingFarZone ? RGBLights.Colors.VIOLET : RGBLights.Colors.YELLOW)));
//        }
//
//        /// REQUEST RAPID FIRE
//        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && !shooter.shooterLatchOpen) {
//            if (turret.targetInRange) {
//                vinWantsToShoot = true;
//                singleShot = false;
//            } else {
//                alertAction(RGBLights.Colors.RED);
//            }
//        }
//        /// REQUEST SINGLE SHOT
//        else if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left && !shooter.shooterLatchOpen) {
//            if (turret.targetInRange) {
//                vinWantsToShoot = true;
//                singleShot = true;
//            } else {
//                alertAction(RGBLights.Colors.RED);
//            }
//        }
//
//        // operate loops (ugly to put here, but fixes small bug)
////        shooter.operateOdomTracking(pinpoint.X, pinpoint.Y, blueAlliance, vinWantsToShoot, cyclingFarZone);
////        turret.operateOdomTracking(pinpoint.X, pinpoint.Y, pinpoint.normalizedHeading, blueAlliance, vinWantsToShoot);
//        shootWhileMoveCalcsSimple();
//
//        // SHOOTER
//        if (vinWantsToShoot) {
//            if (shooter.atTargetRPM && turret.atTargetAngle && !shooter.shooterLatchOpen) {
//                if (singleShot) {
//                    /// SINGLE SHOT
//                    useManualIntake = false;
//                    runningActions.add(new SequentialAction(
//                            new InstantAction(() -> intake.idle()),
//                            new InstantAction(() -> shooter.openLatch()),
//                            new InstantAction(() -> intake.deployBallPusher()),
//                            new SleepAction(PUSHER_DELAY),
//                            new InstantAction(() -> intake.stowBallPusher()),
//                            new InstantAction(() -> shooter.closeLatch()),
//                            new InstantAction(() -> useManualIntake = true),
//                            new InstantAction(() -> singleShot = false),
//                            new InstantAction(() -> vinWantsToShoot = false)
//                    ));
//                } else {
//                    /// RAPID FIRE
//                    useManualIntake = false;
//                    runningActions.add(new SequentialAction(
//                            new SleepAction(0.03),
//                            new InstantAction(() -> shooter.openLatch()),
//                            new InstantAction(() -> intake.intakeFullPower()),
//                            new SleepAction(RAPID_FIRE_DELAY),
//                            new InstantAction(() -> intake.deployBallPusher()),
//                            new SleepAction(PUSHER_DELAY),
//                            new InstantAction(() -> intake.stowBallPusher()),
//                            new InstantAction(() -> shooter.closeLatch()),
//                            new InstantAction(() -> intake.idle()),
//                            new InstantAction(() -> useManualIntake = true),
//                            new InstantAction(() -> vinWantsToShoot = false)
//                    ));
//                }
//            }
//        }
//
//        // intake logic
//        if (useManualIntake) {
//            if (currentGamepad1.right_trigger > 0.9 && intake.intakeState != Intake.IntakeState.INTAKING_FULL_POWER) {
//                intake.intakeFullPower();
//            } else if (currentGamepad1.right_trigger > 0.1 && intake.intakeState != Intake.IntakeState.INTAKING) {
//                intake.intake();
//            } else if (currentGamepad1.a && !previousGamepad1.a && intake.intakeState != Intake.IntakeState.REVERSE) {
//                intake.reverse();
//            } else if (!currentGamepad1.a && previousGamepad1.a && intake.intakeState == Intake.IntakeState.REVERSE) {
//                intake.idle();
//            } else if (currentGamepad1.right_trigger <= 0.1 && intake.intakeState != Intake.IntakeState.IDLE && intake.intakeState != Intake.IntakeState.REVERSE) {
//                intake.idle();
//            }
//        }
//        intake.operateCurrentLimiting();
//
//        // Reset functions
//        if (currentGamepad1.left_trigger > 0.8) {
//            // limelight pose reset
//            llVision.trackPose(blueAlliance);
//            if (llVision.tagSeen) {
//                Pose currentLLPose = llVision.absRelocalize(Math.toRadians(pinpoint.normalizedHeading));
//                if (currentGamepad1.y && !previousGamepad1.y) {
//                    if (currentLLPose.getX() == 0 || currentLLPose.getY() == 0) {
//                        alertAction(RGBLights.Colors.YELLOW);
//                    } else {
//                        pinpoint.teleOpAprilTagReset(currentLLPose);
//                        alertAction(RGBLights.Colors.GREEN);
//                    }
//                }
//                telemetry.addData("LL X", currentLLPose.getX());
//                telemetry.addData("LL Y", currentLLPose.getY());
//            }
//
//            // manual pinpoint heading reset
//            if (currentGamepad1.b && !previousGamepad1.b) {
//                pinpoint.teleOpResetHeading();
//                alertAction(RGBLights.Colors.GREEN);
//            }
//        }
//
//        // manual driver 2 goal pose adjust
//        if (currentGamepad2.right_bumper) {
//            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
//                adjustGoalPose(false, 1);
//                microAlert(RGBLights.Colors.GREEN);
//            } else if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
//                adjustGoalPose(false, -1);
//                microAlert(RGBLights.Colors.GREEN);
//            } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
//                adjustGoalPose(true, 1);
//                microAlert(RGBLights.Colors.BLUE);
//            } else if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
//                adjustGoalPose(true, -1);
//                microAlert(RGBLights.Colors.BLUE);
//            } else if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
//                // full reset
//                Globals.blueGoalX = 10.5;
//                Globals.blueGoalY = 139;
//                Globals.redGoalX = 134.5;
//                Globals.redGoalY = 139;
//            }
//        }
//
//        // telemetry
//        telemetry.addLine("\nOTHER");
//        telemetry.addLine(blueAlliance ? "BLUE ALLIANCE" : "RED ALLIANCE");
//        telemetry.addData("cyclingFarZone", cyclingFarZone);
//        telemetry.addData("Loop Times", elapsedtime.milliseconds());
//
////        if (blueAlliance) {
////            telemetry.addData("Blue Goal X", Globals.blueGoalX);
////            telemetry.addData("Blue Goal Y", Globals.blueGoalY);
////        } else {
////            telemetry.addData("Red Goal X", Globals.redGoalX);
////            telemetry.addData("Red Goal Y", Globals.redGoalY);
////        }
////        telemetry.addData("LL X", currentLLPose.getX());
////        telemetry.addData("LL Y", currentLLPose.getY());
////        telemetry.addData("LL heading (deg)", Math.toDegrees(currentLLPose.getHeading())); // should match pinpoint coords
//        telemetry.addLine("\nPOSE"); // TODO: delete below when working correctly
//        telemetry.addData("pp X", pinpoint.X);
//        telemetry.addData("pp Y", pinpoint.Y);
//        telemetry.addData("pp heading (deg)", pinpoint.normalizedHeading);
//        elapsedtime.reset();
//    }
//
//    @Override
//    public void stop() {
//        Globals.blueGoalX = 10.5;
//        Globals.blueGoalY = 139;
//        Globals.redGoalX = 134.5;
//        Globals.redGoalY = 139;
//    }
//
//    private void alertAction(RGBLights.Colors alertColor) {
//        runningActions.add(new SequentialAction(
//                new InstantAction(() -> lights.setColor(alertColor)),
//                new SleepAction(RGB_ALERT_DELAY),
//                new InstantAction(() -> lights.setColor(RGBLights.Colors.WHITE))
//        ));
//    }
//
//    private void microAlert(RGBLights.Colors alertColor) {
//        runningActions.add(new SequentialAction(
//                new InstantAction(() -> lights.setColor(alertColor)),
//                new SleepAction(0.2),
//                new InstantAction(() -> lights.setColor(RGBLights.Colors.WHITE))
//        ));
//    }
//
//    private void adjustGoalPose(boolean changeX, int dir) {
//        if (changeX) {
//            if (blueAlliance) {
//                Globals.blueGoalX = Globals.blueGoalX + dir;
//            } else {
//                Globals.redGoalX = Globals.redGoalX + dir;
//            }
//        } else {
//            if (blueAlliance) {
//                Globals.blueGoalY = Globals.blueGoalY + dir;
//            } else {
//                Globals.redGoalY = Globals.redGoalY + dir;
//            }
//        }
//    }
//
//    private void shootWhileMoveCalcs() {
//        double temp_time = elapsedtime.milliseconds();
//
//        double current_x_dist = (blueAlliance ? Globals.blueGoalX : Globals.redGoalX) - pinpoint.X;
//        double current_y_dist = (blueAlliance ? Globals.blueGoalY : Globals.redGoalY) - pinpoint.Y;
//
//        Vector robotToGoalVector = new Vector();
//        robotToGoalVector.setOrthogonalComponents(current_x_dist, current_y_dist);
//
//        double currentDist = robotToGoalVector.getMagnitude();
//        double staticCalcRPM = shooter.distanceToRPM(currentDist);
//        double launchAngle = shooter.targetHoodAngle;
//        double horizontalLaunchVel = (480 * Math.log10(Math.log10(launchAngle) * staticCalcRPM))-1509; // derived empirically, don't question, in inches/sec
//
//        Vector robotVelocity = pinpoint.velVector;
//        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();
//        double driveParallelVel = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
//        double drivePerpVel = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();
//
//        double airtime = currentDist / horizontalLaunchVel;
//        double ballParallelVel = currentDist / airtime + driveParallelVel;
//        double adjustedTotalBallVel = Math.hypot(ballParallelVel, drivePerpVel);
//        double adjustedDist = adjustedTotalBallVel * airtime;
//
//        double true_target_heading = Math.toDegrees(Math.atan2(current_y_dist, current_x_dist));
//        double angleOffset = Math.toDegrees(Math.atan2(drivePerpVel, ballParallelVel));
//        double turretAdjustedTargetAngle = normalize(true_target_heading - pinpoint.normalizedHeading - angleOffset); /// not sure + or - angleoffset
//
//        // MIGHT USE DIFFERLE + RPM THAT RELIES ON HOOD MORE LATER
//        shooter.operateShootWhileMove(vinWantsToShoot, adjustedDist, cyclingFarZone);
//        turret.operateShootWhileMove(turretAdjustedTargetAngle);
//
//        /// TELEMETRY
//        telemetry.addLine("\n=== SHOOT WHILE MOVE CALCS ===");
////        telemetry.addData("processing time taken", elapsedtime.milliseconds() - temp_time);
//        telemetry.addData("current dist", currentDist);
//        telemetry.addData("adjustedDist", adjustedDist);
////        telemetry.addData("true_target_heading", true_target_heading);
////        telemetry.addData("angleOffset", angleOffset);
////        telemetry.addData("turretAdjustedTargetAngle", turretAdjustedTargetAngle);
////        telemetry.addData("staticCalcRPM", staticCalcRPM);
////        telemetry.addData("launchAngle (from vert)", launchAngle);
////        telemetry.addData("log(log(angle)*RPM)", Math.log10(Math.log10(launchAngle) * staticCalcRPM));
////        telemetry.addData("horizontalLaunchVel", horizontalLaunchVel);
////        telemetry.addData("robotVelocity theta", robotVelocity.getTheta());
////        telemetry.addData("coordinateTheta", coordinateTheta);
////        telemetry.addData("driveParallelVel", driveParallelVel);
////        telemetry.addData("drivePerpVel", drivePerpVel);
//        telemetry.addData("airtime", airtime);
////        telemetry.addData("ballParallelVel", ballParallelVel);
////        telemetry.addData("adjustedTotalBallVel", adjustedTotalBallVel);
//
//    }
//
//    private void shootWhileMoveCalcsSimple() {
//        double temp_time = elapsedtime.milliseconds();
//
//        double currentXDist = (blueAlliance ? Globals.blueGoalX : Globals.redGoalX) - pinpoint.X;
//        double currentYDist = (blueAlliance ? Globals.blueGoalY : Globals.redGoalY) - pinpoint.Y;
//
//        double currentDist = Math.hypot(currentXDist, currentYDist);
//
//        telemetry.addLine("\n=== SHOOT WHILE MOVE CALCS ===");
//
//        if (Math.abs(pinpoint.velX) < 0.1 && Math.abs(pinpoint.velY) < 0.1) {
//            shooter.operateShootWhileMoveSimple(currentXDist, currentYDist, vinWantsToShoot, cyclingFarZone);
//            turret.operateShootWhileMoveSimple(currentXDist, currentYDist, pinpoint.normalizedHeading);
//
//            telemetry.addLine("0");
//            telemetry.addLine("0");
//            telemetry.addLine("0");
//        }
//        else {
//            double airtime = -0.0000188859 * Math.pow(currentDist, 2) + 0.00653368 * currentDist + 0.1173094;
//            double adjustedXDist = currentXDist - (pinpoint.velX * airtime);
//            double adjustedYDist = currentYDist - (pinpoint.velY * airtime);
//
//            shooter.operateShootWhileMoveSimple(adjustedXDist, adjustedYDist, vinWantsToShoot, cyclingFarZone);
//            turret.operateShootWhileMoveSimple(adjustedXDist, adjustedYDist, pinpoint.normalizedHeading);
//
//            telemetry.addData("airtime", airtime);
//            telemetry.addData("adjustedXDist", adjustedXDist);
//            telemetry.addData("adjustedYDist", adjustedYDist);
//        }
//
//        telemetry.addData("pinpoint.velX", pinpoint.velX);
//        telemetry.addData("pinpoint.velY", pinpoint.velY);
//        telemetry.addData("processing time taken", elapsedtime.milliseconds() - temp_time);
//    }
//
//    private double normalize(double angle) {
//        while (angle > 180) angle -= 360;
//        while (angle < -180) angle += 360;
//        return angle;
//    }
//}
