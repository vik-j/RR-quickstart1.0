package org.firstinspires.ftc.teamcode.teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.pathgen.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auton.qolActions.qol;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Stack;
import java.util.Vector;

@Config
//TODO: change claw opened and closed values
public class Robot {
//    public DcMotor leftFront, leftBack, rightFront, rightBack;
    public DcMotor flip, slide;
    public Servo wrist, leftHang, rightHang;
    public CRServo intakeLeft, intakeRight;
    public Servo grippy, twisty, flippy, touchy, sweepy;
    public MecanumDrive drive;
    public PIDController armController, slideController;
    public DistanceSensor lookyLeft, lookyRight;
    public Limelight3A limelight;
    public RevBlinkinLedDriver leds;
    public double epsilon = 0.1;
    public boolean endPID = false;

    public static double robotX = 0, robotY = 0, robotH = 0;

    public PIDCoefficients x = new PIDCoefficients(0,0,0);
    public PIDCoefficients h = new PIDCoefficients(0,0,0);
    public double lateralMultiplier = 1;
    public int rightBumperCounter = 0;

    public static double xA = -0.04556333;
    public static double xB = -7.3511622E-8;
    public static double xC = 7.8536214;
    public static double xD = 118.42621;

    public static double yA = 0.001160505;
    public static double yB = -2.387352E-7;
    public static double yC = -4.712338;
    public static double yD = -9.22812;

    public HardwareMap hardwareMap;

    public final double gripClawOpen = 0, gripClawClosed = 0.1;
    public double flipPos, slidePos;
    public int slideExtensionLimit = 1100;
    public int armTarget = 0, slideTarget = 0;
    public int armTargetAuto = 0, slideTargetAuto = 0;
    public static volatile boolean stopPid = false;
    public double wristTargetAuto = 0.0;
    public double intakeMultiplier = 1;
    public static double leftHangPosUp = 0.9, leftHangPosDown = 0.4, rightHangPosUp = 0.9, rightHangPosDown = 0.4;
    Thread currentThread = null;

    public static final double slidesTTS = 3260 / 20.0;

    //TODO: claw extends 2.25 inches away from drivetrain
    //TODO: slides tts is 3260 tick per 20 inch

    public double pivotMultiplier = 1;

    public final double noTimeout = -1.0;

    public static Pose2d lastPose = new Pose2d(0,0,0);

    Gamepad gamepad1, gamepad2;

    public static class CompFieldOffsets {
        public static double Gold1Red = 0.8759407545;
        public static double Gold1Blue = 0.8759407545;
        public static double Gold2Red = 0.8759407545;
        public static double Gold2Blue = 0.7808369606;

        public static double Silicon1Red = 0.8108696933;
        public static double Silicon1Blue = 0.5605981613;
        public static double Silicon2Red = 0.4808369606;
        public static double Silicon2Blue = 0.7508042686;
    }
    public static Action driveAction(MecanumDrive drive, Pose2d beginPose, qol q, double wallOffset) {
        return drive.actionBuilder(beginPose)
                //TODO: 1st speci

                .afterTime(0.01, q.firstSpeci())
                .strafeToConstantHeading(new Vector2d(-8.5, 34.65 + wallOffset))
                .afterTime(0, q.firstSpeci2())
                .waitSeconds(0.2)
                .afterTime(0, q.grippyOpen())
                .afterTime(0.1, q.combine(q.reset(), q.flippy(1)))
                .afterTime(1, q.autoSamplePickup())
                .splineToSplineHeading(new Pose2d(-27.85,41.25, Math.toRadians(-145.5)), Math.toRadians(180))

                //TODO: pickup 1st sample
                // .splineToSplineHeading(new Pose2d(-28.07,39.92, Math.toRadians(-145.5)), Math.toRadians(180))
                .afterTime(0.2, q.flippy(0.4))
                .afterTime(0.42, q.grippyClose())
                .afterTime(0.9, q.flippy(0.6))
                .waitSeconds(0.5)

                //TODO: drop off first sample
                .turnTo(Math.toRadians(123.5), new TurnConstraints(20, -20, 20))
                .afterTime(0, q.grippyOpen())
                .splineToSplineHeading(new Pose2d(-38.1, 41.28, Math.toRadians(-146.44)), Math.toRadians(270))

                //TODO: pick up 2nd sample
                //.splineToSplineHeading(new Pose2d(-39.15, 40.48, Math.toRadians(-146.44)), Math.toRadians(270))
                .afterTime(0.25, q.flippy(0.4))
                .afterTime(0.47, q.grippyClose())
                .afterTime(0.9, q.flippy(0.6))

                //TODO: drop off 2nd sample
                .waitSeconds(0.55)
                .turnTo(Math.toRadians(121.95), new TurnConstraints(20, -20, 20))
                .afterTime(0, q.combine(q.grippyOpen(), new InstantAction(() -> drive.setCorrectionTimeout(1.25))))
                .splineToSplineHeading(new Pose2d(-46.85, 41.3, Math.toRadians(-151.7)), Math.toRadians(270))

                //TODO: pick up 3rd sample
                // .splineToSplineHeading(new Pose2d(-45.97, 39.097, Math.toRadians(-151.7)), Math.toRadians(270))
                .afterTime(0.3, q.flippy(0.4))
                .afterTime(0.52, q.combine(q.grippyClose(), new InstantAction(() -> drive.setCorrectionTimeout(1))))
                .afterTime(1.05, q.combine(q.twisty(0.75), q.flippy(0.6)))
                .afterTime(1.3, q.combine(q.reset(), q.flippy(0.6)))
                .waitSeconds(0.65)
                .afterTime(0.6, q.arm(0, 800))

                //TODO: drop off 3rd sample
                .strafeToSplineHeading(new Vector2d(-38.2, 48.193), Math.toRadians(120.3))
                .afterTime(0, q.grippyOpen())


                .afterTime(0.75, q.specimenPickup())
                .waitSeconds(0.5)
                //TODO: pickup 2nd speci
                .strafeToLinearHeading(new Vector2d(-35.52, 47), Math.toRadians(-90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-35.52, 54), Math.toRadians(-90))
                .afterTime(0.25, q.grippyClose())
                .afterTime(0.7, q.flippy(0.8))
                .waitSeconds(0.15)
                .afterTime(0, q.flippy(0.9))
                .afterTime(0.3, q.combine(q.speciScoreReset(), q.flippy(0.9)))
                .afterTime(1.05, q.specimenDeposit())
                //TODO: score 2nd speci
                .strafeToConstantHeading(new Vector2d(-3, 35.15 + wallOffset))

                .afterTime(0.2, q.specimenDeposit2())
                .afterTime(0.5, q.grippyOpen())
                .waitSeconds(0.1)


                .afterTime(0.75, q.specimenPickup())
                //TODO: pickup 3rd speci
                .strafeToLinearHeading(new Vector2d(-35.52, 50), Math.toRadians(-87.5))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-35.52, 53.5), Math.toRadians(-90))
                .afterTime(0, q.grippyClose())
                .afterTime(0.6, q.flippy(0.8))
                .waitSeconds(0)
                .afterTime(0, q.flippy(0.9))
                .afterTime(0.35, q.combine(q.speciScoreReset(), q.flippy(0.9)))
                .afterTime(1.2, q.specimenDeposit())

                //TODO: score 3rd speci
                .strafeToConstantHeading(new Vector2d(0, 34.18 + wallOffset))

                .afterTime(0.2, q.specimenDeposit2())
                .afterTime(0.45, q.grippyOpen())
                .waitSeconds(0.1)


                .afterTime(0.75, q.specimenPickup())

                //TODO: pick up 4th speci
                .strafeToLinearHeading(new Vector2d(-35.52, 50), Math.toRadians(-87.5))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-35.52, 53), Math.toRadians(-90))
                .afterTime(0, q.grippyClose())
                .afterTime(0.6, q.flippy(0.7))
                .waitSeconds(0)
                .afterTime(0, q.flippy(0.9))
                .afterTime(0.4, q.combine(q.speciScoreReset(), q.flippy(0.9)))
                .afterTime(1.37, q.specimenDeposit())
                //TODO: drop off 4th speci
                .strafeToConstantHeading(new Vector2d(5, 34.32 + wallOffset))
                .afterTime(0.3, q.specimenDeposit2())
                .afterTime(0.65, q.grippyOpen())
                .waitSeconds(0.12)


                .afterTime(0.75, q.specimenPickup())


                //TODO: pick up 5th speci
                .strafeToLinearHeading(new Vector2d(-35.52, 50), Math.toRadians(-87.5))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-35.52, 53.3), Math.toRadians(-90))
                .afterTime(0, q.grippyClose())
                .afterTime(0.6, q.flippy(0.7))
                .waitSeconds(0)
                .afterTime(0, q.flippy(0.9))
                .afterTime(0.4, q.combine(q.speciScoreReset(), q.flippy(0.9)))
                .afterTime(1.32, q.specimenDeposit())
                //TODO: drop off 5th speci
                .strafeToConstantHeading(new Vector2d(7, 34 + wallOffset))
                .afterTime(0.15, q.specimenDeposit2())
                .afterTime(0.47 , q.grippyOpen())
                .waitSeconds(0.2)
                .afterTime(0, q.hangUp())
                .afterTime(0.2, q.hangAlmostDown())
                .afterTime(0.4, q.hangUp())

//                .afterTime(2, q.reset())

                //TODO: park
//                .strafeToConstantHeading(new Vector2d(-60, 60), new TranslationalVelConstraint(120), new ProfileAccelConstraint(-120, 120))


                .build();
    }

    public static class DriveAction implements Action{
        qol q;
        double wallOffset;
        MecanumDrive drive;

        public DriveAction(MecanumDrive drive, qol q, double wallOffset) {
            this.q = q;
            this.wallOffset = wallOffset;
            this.drive = drive;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            return false;
        }
    }

    public Robot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        this.hardwareMap = hardwareMap;

//        leftFront = hardwareMap.dcMotor.get("leftFront");
//        leftBack = hardwareMap.dcMotor.get("leftBack");
//        rightFront = hardwareMap.dcMotor.get("rightFront");
//        rightBack = hardwareMap.dcMotor.get("rightBack");

        flip = hardwareMap.dcMotor.get("flip");
        slide = hardwareMap.dcMotor.get("slide");

//        leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");

        grippy = hardwareMap.servo.get("claw");
        twisty = hardwareMap.servo.get("twist");
        flippy = hardwareMap.servo.get("flippy");
        touchy = hardwareMap.servo.get("touchy");
        sweepy = hardwareMap.servo.get("sweepy");

        leftHang = hardwareMap.servo.get("leftHang");
        rightHang = hardwareMap.servo.get("rightHang");

//        lookyLeft = hardwareMap.get(DistanceSensor.class, "lookyLeft");
//        lookyRight = hardwareMap.get(DistanceSensor.class, "lookyRight");
//        List<DcMotor> motors = Arrays.asList(leftBack, leftFront, rightBack, rightFront, flip, slide);

//        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        flip.setDirection(DcMotorSimple.Direction.FORWARD);
        flip.setDirection(DcMotorSimple.Direction.FORWARD);

        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        twisty.setDirection(Servo.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

//        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);


//        for (DcMotor motor: motors) {
//            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        }
//
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armController = new PIDController(armPIDValues.fP, armPIDValues.fI, armPIDValues.fD);
        slideController = new PIDController(armPIDValues.sP,armPIDValues.sI,armPIDValues.sD);

        hangUp();
    }


    public void setTelemToDashboard(Telemetry telem) {
        telem = new MultipleTelemetry(telem, FtcDashboard.getInstance().getTelemetry());
    }

    public void arcadeDrive(Gamepad gamepad1) {
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -0.75*gamepad1.right_stick_x;

//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double leftFrontPower = (y + x + rx) / denominator;
//        double leftBackPower = (y - x + rx) / denominator;
//        double rightFrontPower = (y - x - rx) / denominator;
//        double rightBackPower = (y + x - rx) / denominator;
//
//        leftFront.setPower(leftFrontPower);
//        leftBack.setPower(leftBackPower);
//        rightFront.setPower(rightFrontPower);
//        rightBack.setPower(rightBackPower);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-y,x), rx));
    }

    public void arcadeDriveWithSlowMode(Gamepad gamepad) {
        double y,x,rx;
        if (gamepad.right_trigger > 0) {
            y = 0.5*gamepad.left_stick_y;
            x = -0.5*gamepad.left_stick_x;
            rx = -0.5*gamepad.right_stick_x;
        }
        else {
            y = gamepad.left_stick_y;
            x = -gamepad.left_stick_x;
            rx = -0.75*gamepad.right_stick_x;
        }

//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double leftFrontPower = (y + x + rx) / denominator;
//        double leftBackPower = (y - x + rx) / denominator;
//        double rightFrontPower = (y - x - rx) / denominator;
//        double rightBackPower = (y + x - rx) / denominator;
//
//        leftFront.setPower(leftFrontPower);
//        leftBack.setPower(leftBackPower);
//        rightFront.setPower(rightFrontPower);
//        rightBack.setPower(rightBackPower);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(x,y), rx));
    }
    public void arcadeDriveWithSlowModeForLittleChildren(Gamepad gamepad) {
        double y,x,rx;
        if (gamepad.right_trigger > 0) {
            y = -0.5*gamepad.left_stick_y;
            x = 0.5*gamepad.left_stick_x;
            rx = 0.5*gamepad.right_stick_x;
        }
        else {
            y = -0.4*gamepad.left_stick_y;
            x = 0.4*gamepad.left_stick_x;
            rx = 0.2*gamepad.right_stick_x;
        }

//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double leftFrontPower = (y + x + rx) / denominator;
//        double leftBackPower = (y - x + rx) / denominator;
//        double rightFrontPower = (y - x - rx) / denominator;
//        double rightBackPower = (y + x - rx) / denominator;
//
//        leftFront.setPower(leftFrontPower);
//        leftBack.setPower(leftBackPower);
//        rightFront.setPower(rightFrontPower);
//        rightBack.setPower(rightBackPower);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(x,y), rx));
    }

    public void slideControl(Gamepad gamepad) {
        slide.setPower(-gamepad.left_stick_y * 0.3);
    }

    public void autoSamplePickup() {
        twisty.setPosition(scaleTwisty(0.75));
        flippy.setPosition(scaleFlippy(0.6));

        Actions.runBlocking(setPidVals(0, 2300));
    }

    public void autoSamplePickup2() {
        twisty.setPosition(scaleTwisty(0.75));
        flippy.setPosition(scaleFlippy(0.6));

        Actions.runBlocking(setPidVals(0, 1800));
    }
    public void newAutoSpeci() {
        flippy.setPosition(scaleFlippy(0.4));

        Actions.runBlocking(setPidVals(1250, 2700));
    }

    public void autoSampleSweeping() {
        twisty.setPosition(scaleTwisty(0.75));
        flippy.setPosition(scaleFlippy(0.65));
        //TODO: Down is 0.42

        Actions.runBlocking(setPidVals(220, 2300));
    }

    public void sweepyUp() {
        sweepy.setPosition(1);
    }
    public void sweepyDown() {
        sweepy.setPosition(0);
    }
    public void touchyTouch() {
        touchy.setPosition(0);
    }
    public void touchyRetract() {
        touchy.setPosition(1);
    }
    public void grippyClose() {
        grippy.setPosition(1);
    }
    public void grippyOpen() {
        grippy.setPosition(0);
    }
    public void wallMacro() {
        flippy.setPosition(scaleFlippy(0.828));

        Actions.runBlocking(setPidVals(915, 1130));
    }
    public void resetEncoders() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void wallMacro2() {
        armTarget = 1360;
        Actions.runBlocking(setPidVals(1360, 1130));
    }
    public void badClose() {
        grippy.setPosition(0.972);
    }
    public void newSpeci() {
//        touchyTouch();
        flippy.setPosition(scaleFlippy(0.828));
//            armTarget = 2120;
//            slideTarget = 1256;

        Actions.runBlocking(setPidVals(885, 1550));
    }
    public void sampleUp() {
        Actions.runBlocking(setPidVals(1900, 2300));
    }
    public ArmPosition speciDeposit() {
        return new ArmPosition(945, 1634, grippy.getPosition(), 0.828, twisty.getPosition(), 0);
    }
    public void newSpeciPivot() {
        touchyRetract();
        flippy.setPosition(scaleFlippy(0.828));
        Actions.runBlocking(setPidVals(945, 0));
    }
    public ArmPosition speciDepositPivot() {
        return new ArmPosition(945, 0, grippy.getPosition(), 0.828, twisty.getPosition(), 1);
    }
    public void newSpeciSlides() {
        touchyTouch();
        flippy.setPosition(scaleFlippy(0.828));
        Actions.runBlocking(setPidVals(915, 1720));
    }
    public ArmPosition speciDepositSlides() {
        return speciDeposit();
    }

    public void newSpeci2() {
        touchyRetract();
        Actions.runBlocking(setPidVals(1360, 1740));
    }

    public ArmPosition speciDeposit2() {
        return new ArmPosition(1360, 1700, grippy.getPosition(), flippy.getPosition(), twisty.getPosition(), touchy.getPosition());
    }
    public void speciMacroPlus(int change) {
        flippy.setPosition(scaleFlippy(0.718));
        twisty.setPosition(scaleTwisty(1));
        Actions.runBlocking(setPidVals(2120, 1050 + change));
    }
    public void specimenPickup() {
        touchyRetract();
        flippy.setPosition(scaleFlippy(0.97));
        Actions.runBlocking(setPidVals(2540,0));
        twisty.setPosition(scaleTwisty(0));
        grippyOpen();
    }
    public void specimenPickupTELE() {
        touchyRetract();
        flippy.setPosition(scaleFlippy(0.97));
        armTarget = 2510;
        slideTarget = 0;
        twisty.setPosition(scaleTwisty(0));
        grippyOpen();
    }
    public void specimenDeposit() {
        flippy.setPosition(scaleFlippy(0.828));

        Actions.runBlocking(setPidVals(945, 1560));
    }
    public void specimenDepositTELE() {
        flippy.setPosition(scaleFlippy(0.828));
        armTarget = 945;
        slideTarget = 1560;
    }
    public void specimenDeposit2() {
        Actions.runBlocking(setPidVals(1420, 1560));
    }
    public void specimenDeposit2TELE() {
        armTarget = 1420;
        slideTarget = 1560;
    }

    public void sampleDeposit() {
        flippy.setPosition(scaleFlippy(0.7));
        twisty.setPosition(scaleTwisty(1));
        Actions.runBlocking(setPidVals(1875, 4700));
    }
    public void samplePivot() {
        flippy.setPosition(scaleFlippy(0.75));
        twisty.setPosition(scaleTwisty(0));
        Actions.runBlocking(setPidVals(1900, 0));
    }
    public void sampleSlides() {
        Actions.runBlocking(setPidVals(1900, 4600));
    }
    public void sampleScore3() {
        Actions.runBlocking(setPidVals(2000, 4700));
    }
    public void speciPickup2() {
        grippyOpen();
        flippy.setPosition(scaleFlippy(0.778));
        twisty.setPosition(scaleTwisty(0));
    }
    public void fullRetract() {
        Actions.runBlocking(setPidVals(0,0));
        twisty.setPosition(scaleTwisty(0));
    }
    public void reset() {
        intakeMultiplier = 1;
        Actions.runBlocking(setPidVals(0,0));
        flippy.setPosition(scaleFlippy(0.4));
        twisty.setPosition(scaleTwisty(0));
    }
    public void resetTELE() {
        armTarget = 0;
        slideTarget = 0;
    }
    public void speciScoreReset() {
        intakeMultiplier = 1;
        Actions.runBlocking(setPidVals(1200,0));
        flippy.setPosition(scaleFlippy(0.4));
        twisty.setPosition(scaleTwisty(0));
    }
    public void speciScoreResetTELE() {
        armTarget = 1200;
        flippy.setPosition(scaleFlippy(0.4));
        twisty.setPosition(scaleTwisty(0));
    }
    public void newNewSpeciScoreTELE() {
        flippy.setPosition(scaleFlippy(0.4));

        armTarget = 1250;
        slideTarget = 2700;
    }
    public ArmPosition retract() {
        return new ArmPosition(0,0,0,0.97,0,1);
    }
    public void samplePickup() {
        flippy.setPosition(scaleFlippy(0.4));
        grippyOpen();
        twisty.setPosition(scaleTwisty(0));
        Actions.runBlocking(setPidVals(175, 1730));
    }
    public void samplePickupPart2() {
        rightBumperCounter = 0;
        Actions.runBlocking(setPidVals(0, 1830));
        armTarget = 0;
        intakeMultiplier = 1;

        grippy.setPosition(1);

        flippy.setPosition(scaleFlippy(1));

        twisty.setPosition(scaleTwisty(0));
        Actions.runBlocking(setPidVals(0,0));
    }

    public boolean withinDistanceLeft(double target) {
        double leftDistance = lookyLeft.getDistance(DistanceUnit.INCH);
        double tolerance = 0.1;

        return (target < leftDistance - tolerance) && (target > leftDistance + tolerance);

    }

    public boolean withinDistanceRight(double target) {
        double rightDistance = lookyRight.getDistance(DistanceUnit.INCH);
        double tolerance = 0.1;

        return (target < rightDistance - tolerance) && (target > rightDistance + tolerance);

    }

    public boolean sameDistance() {
        double leftDistance = lookyLeft.getDistance(DistanceUnit.INCH);
        double rightDistance = lookyRight.getDistance(DistanceUnit.INCH);
        double tolerance = 0.1;

        return (rightDistance < leftDistance - tolerance) && (rightDistance > leftDistance + tolerance);

    }

    public boolean checkDistance(double goal) {
        double leftDistance = lookyLeft.getDistance(DistanceUnit.INCH);
        double rightDistance = lookyRight.getDistance(DistanceUnit.INCH);

        return (sameDistance()) && (withinDistanceLeft(goal)) && (withinDistanceRight(goal));
    }

    public double findAngle() {
        double sensorDifference = 6;
        double leftDistance = lookyLeft.getDistance(DistanceUnit.INCH);
        double rightDistance = lookyRight.getDistance(DistanceUnit.INCH);

        double realOffset = leftDistance-rightDistance;
        double offset = Math.abs(leftDistance-rightDistance);

        double hypotenuse = Math.sqrt(Math.pow(offset,2) +Math.pow(sensorDifference,2));
        double robotGeneralAngle = 90*(Math.round(Math.toDegrees(drive.pose.heading.toDouble())/90));

        double trueAngle;

        if(realOffset < 0){
            double offsetAngle = -Math.asin(offset/hypotenuse);

            trueAngle = robotGeneralAngle + offsetAngle;

            return trueAngle;
        } else if (realOffset >= 0) {
            double offsetAngle = Math.asin(offset/hypotenuse);

            trueAngle = robotGeneralAngle + offsetAngle;

            return trueAngle;
        } else{
            return 0;
        }
    }

    public double averageVisionDistance(){
        double leftDistance = lookyLeft.getDistance(DistanceUnit.INCH);
        double rightDistance = lookyRight.getDistance(DistanceUnit.INCH);

        return (leftDistance+rightDistance)/2;
    }

    public double startingXValue(){
        double sensorDifference = 6;
        double leftDistance = lookyLeft.getDistance(DistanceUnit.INCH);
        double rightDistance = lookyRight.getDistance(DistanceUnit.INCH);

        double offset = Math.abs(leftDistance-rightDistance);

        double hypotenuse = Math.sqrt(Math.pow(offset,2) +Math.pow(sensorDifference,2));
        double angle = Math.asin(offset/hypotenuse);

        double distanceFromCenter = ((2*leftDistance/3)+8);

        double XValue = Math.cos(angle)*distanceFromCenter;

        return XValue;

    }

    public double startingYValue(){
        double sensorDifference = 6;
        double leftDistance = lookyLeft.getDistance(DistanceUnit.INCH);
        double rightDistance = lookyRight.getDistance(DistanceUnit.INCH);

        double offset = Math.abs(leftDistance-rightDistance);

        double hypotenuse = Math.sqrt(Math.pow(offset,2) +Math.pow(sensorDifference,2));
        double angle = Math.asin(offset/hypotenuse);

        double distanceFromCenter = ((2*leftDistance/3)+8);

        double YValue = Math.sin(angle)*distanceFromCenter;

        return YValue;

    }

    public void newSpeciScoreAutomated() {
        endPID = false;
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, Math.toRadians(270)));
        Actions.runBlocking(new ParallelAction(drive.actionBuilder(new Pose2d(0,0,Math.toRadians(270)))
                .afterTime(0.1, telemetryPacket -> {
                    grippyClose();
                    return false;
                })
                .afterTime(0.7, telemetryPacket -> {
                    flippy.setPosition(scaleFlippy(0.8));
                    return false;
                })
                .waitSeconds(0.15)
                .afterTime(0.8, telemetryPacket -> {
                    newNewSpeciScoreTELE();
                    return false;
                })
                //TODO: score 2nd speci
                .splineToConstantHeading(new Vector2d(-4 + 35.52 + 4, 34.79 - 54), Math.toRadians(270))
                .afterTime(0, telemetryPacket -> {endPID = true; return false;})
                .build(), returnCancelableTelePID()));
    }
    public void newSpeciPickupAutomated() {
        endPID = false;
        Actions.runBlocking(new ParallelAction(drive.actionBuilder(new Pose2d(-4 - 35.52, 34.79 - 54, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(0, -4), Math.toRadians(90))
                .afterTime(0, telemetryPacket -> {endPID = true; return false;})
                .build(), returnCancelableTelePID()));
    }
    public void speciScoreAutomated() {
        endPID = false;
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, Math.toRadians(270)));
        Actions.runBlocking(new ParallelAction(drive.actionBuilder(new Pose2d(0,0,Math.toRadians(270)))
                .afterTime(0.1, telemetryPacket -> {
                    badClose();
                    return false;
                })
                .afterTime(0.7, telemetryPacket -> {
                    flippy.setPosition(scaleFlippy(0.8));
                    return false;
                })
                .waitSeconds(0.15)
                .afterTime(0, telemetryPacket -> {
                    flippy.setPosition(scaleFlippy(0.9));
                    return false;
                })
                .afterTime(0.3, telemetryPacket -> {
                    speciScoreResetTELE();
                    flippy.setPosition(scaleFlippy(0.9));
                    return false;
                })
                .afterTime(1, telemetryPacket -> {
                    specimenDepositTELE();
                    return false;
                })
                //TODO: score 2nd speci
                .strafeToConstantHeading(new Vector2d(-4 + 35.52 + 4, 34.79 - 54))
                .afterTime(0, telemetryPacket -> {endPID = true; return false;})
                .build(), returnCancelableTelePID()));
    }
    public Action waitForArmToReset() {
        return telemetryPacket -> {
            telemetryPacket.put("bluh", "bluh");
            return (Math.abs(armTarget) > 10) && (Math.abs(slideTarget) > 100);
        };
    }
    public void speciScoreAndSample() {
        Actions.runBlocking(new SequentialAction(new InstantAction(this::specimenDeposit2TELE)
        , new ParallelAction(new SleepAction(0.3), returnTelePid(0.3)), new InstantAction(this:: grippyOpen),
                new InstantAction(this::reset)));
    }
    public void speciPickupAutomated() {
        endPID = false;
        Actions.runBlocking(new ParallelAction(drive.actionBuilder(new Pose2d(-4 - 35.52, 34.79 - 54, Math.toRadians(270)))
                .afterTime(0, telemetryPacket -> {
                    specimenDeposit2TELE();
                    return false;
                })
                .afterTime(0.4, telemetryPacket -> {grippyOpen(); return false;})
                .waitSeconds(0.5)
                .afterTime(0, new InstantAction(this::specimenPickupTELE))
                //TODO: pickup 3rd speci
                .strafeToLinearHeading(new Vector2d(0, -4), Math.toRadians(-90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))
                .afterTime(0, telemetryPacket -> {endPID = true; return false;})
                .build(), returnCancelableTelePID()));
    }
    public void hangUp() {
        leftHang.setPosition(1);
        rightHang.setPosition(0);
    }
    public void hangDown() {
        leftHang.setPosition(0);
        rightHang.setPosition(1);
    }
    public void hang1() {
        armTarget = 2120;
        flippy.setPosition(scaleFlippy(0.4));
        grippyOpen();
        slideTarget = 1600;
        hangUp();
    }
    public void hang2() {
        slideTarget = 625;
        hangDown();

        Actions.runBlocking(new ParallelAction(new SleepAction(0.75), returnTelePid(0.75)));
        armTarget = 1650;
    }
    public void hang3() {
        armTarget = 1600;
        slideTarget = 4100;
    }
    public void hang4() {
        armTarget = 2000;
        slideTarget = 3800;
    }
    public void hang5() {
        slideTarget = 1500;
        armTarget = 1600;

        Actions.runBlocking(new ParallelAction(new SleepAction(0.3), returnTelePid(0.3)));
        hangUp();
    }
    public void hang6() {
        armTarget = 900;
        slideTarget = 0;
    }
    public void hangCheatCode(Gamepad gamepad2) {
        if (gamepad2.a && gamepad2.right_trigger > 0) {
            hang1();
        }
        else if (gamepad2.b && gamepad2.right_trigger > 0) {
            hang2();
        }
        else if (gamepad2.x && gamepad2.right_trigger > 0) {
            hang3();
        }
        else if (gamepad2.y && gamepad2.right_trigger > 0) {
            hang4();
        }
        else if (gamepad2.dpad_up && gamepad2.right_trigger > 0) {
            hang5();
        }
        else if (gamepad2.dpad_down && gamepad2.right_trigger > 0) {
            hang6();
        }
        else if (gamepad2.dpad_left && gamepad2.right_trigger > 0) {
            slideTarget += 10;
        }
        else if (gamepad2.dpad_right && gamepad2.right_trigger > 0) {
            slideTarget -= 10;
        }
    }
    public void hangAlmostDown() {
        leftHang.setPosition(0.47);
        rightHang.setPosition(0.45);
    }

    public void extendIntoSub(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad2.x) {
            touchyRetract();
            rightBumperCounter = 0;
            slideTarget = 2300;
            armTarget = 240;
            intakeMultiplier = 1;
            flippy.setPosition(scaleFlippy(0.4));
            while (Math.abs(slideTarget - slide.getCurrentPosition()) > 500) {
                TeleopPID(gamepad2);
                arcadeDrive(gamepad1);
            }

            twisty.setPosition(scaleTwisty(0));
            grippy.setPosition(0);
        }
    }
    public void limelightStart() {
        limelight.start();
    }
    public static double normalizeRadToQ1(double input) {
        double normalized = AngleUnit.normalizeRadians(input);

        if (normalized < 0) {
            normalized = -normalized;
        }
        if (normalized > Math.PI / 2) {
            normalized = Math.PI - normalized;
        }
        return normalized;
    }
    public static double sampleMath(List<Vector2d> poses) {
        List<Double> angles = new ArrayList<>();
        List<Double> distances = new ArrayList<>();

        if (poses.size() == 4) {
            Map<Double, Double> angleToDistance = new HashMap<>();

            // Iterate over unique pairs to compute angles and distances
            for (int i = 0; i < poses.size(); i++) {
                for (int j = i + 1; j < poses.size(); j++) {
                    double deltaX = poses.get(j).x - poses.get(i).x;
                    double deltaY = poses.get(j).y - poses.get(i).y;
                    double angle = Math.atan2(deltaY, deltaX);
                    double normalizedAngle = normalizeRadToQ1(angle);
                    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

                    angles.add(normalizedAngle);
                    distances.add(distance);

                    // Map each normalized angle to its distance
                    angleToDistance.put(normalizedAngle, distance);
                }
            }

            List<Double> duplicates = new ArrayList<>();
            double epsilon = 1e-2;

            for (int i = 0; i < angles.size(); i++) {
                for (int j = i + 1; j < angles.size(); j++) {
                    if (Math.abs(angles.get(i) - angles.get(j)) < epsilon && !duplicates.contains(angles.get(i))) {
                        duplicates.add(angles.get(i));
                    }
                }
            }

            double widthwiseAngle = duplicates.get(0);
            double minDistance = Double.MAX_VALUE;

            for (double dupAngle : duplicates) {
                double dist = angleToDistance.get(dupAngle);
                if (dist < minDistance) {
                    minDistance = dist;
                    widthwiseAngle = dupAngle;
                }
            }

            double lengthwiseAngle = widthwiseAngle + (Math.PI / 2);

            return normalizeRadToQ1(lengthwiseAngle);
        }

        return angles.get(0);
    }
    public double getSampleAngle() {
        LLResult result = limelight.getLatestResult();
        List<Double> angles = new ArrayList<>();
        List<Double> distances = new ArrayList<>();
        if (result != null && result.isValid()) {
            List<List<Double>> corners = result.getColorResults().get(0).getTargetCorners();

            List<Vector2d> poses = new ArrayList<>();

            for (List<Double> point: corners) {
                Vector2d vector = new Vector2d(point.get(0), point.get(1));
                poses.add(vector);
            }

            if (poses.size() == 4) {
                Map<Double, Double> angleToDistance = new HashMap<>();

                for (int i = 0; i < poses.size(); i++) {
                    for (int j = i + 1; j < poses.size(); j++) {
                        double deltaX = poses.get(j).x - poses.get(i).x;
                        double deltaY = poses.get(j).y - poses.get(i).y;
                        double angle = Math.atan2(deltaY, deltaX);
                        double normalizedAngle = normalizeRadToQ1(angle);
                        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

                        angles.add(normalizedAngle);
                        distances.add(distance);

                        angleToDistance.put(normalizedAngle, distance);
                    }
                }

                List<Double> duplicates = new ArrayList<>();
                double epsilon = 1e-1;

                for (int i = 0; i < angles.size(); i++) {
                    for (int j = i + 1; j < angles.size(); j++) {
                        if (Math.abs(angles.get(i) - angles.get(j)) < epsilon && !duplicates.contains(angles.get(i))) {
                            duplicates.add(angles.get(i));
                        }
                    }
                }

                double widthwiseAngle = 0;
                double minDistance = Double.MAX_VALUE;

                for (double dupAngle : duplicates) {
                    double dist = angleToDistance.get(dupAngle);
                    if (dist < minDistance) {
                        minDistance = dist;
                        widthwiseAngle = dupAngle;
                    }
                }

                double lengthwiseAngle = widthwiseAngle + (Math.PI / 2);

                return normalizeRadToQ1(lengthwiseAngle);
            }
        }
        return 0;
    }
    public double convertToClawDegree(double radians) {
        return Math.round(Math.toDegrees(normalizeRadToQ1(radians)));
    }
    public double normalizeToQ2Q3(double degrees) {
        degrees = ((degrees % 360) + 360) % 360;

        if (degrees < 90) {
            degrees += 180;
        }
        else if (degrees > 270) {
            degrees -= 180;
        }

        return degrees;
    }
    public double convertDegreesToTwisty(double degrees) {
        double normalized = normalizeToQ2Q3(degrees);
        if (normalized == 270) return 0;
        else return (normalized - 90) / 180;
    }
    public void hangReleaseString() {
        hangAlmostDown();
        Actions.runBlocking(new ParallelAction(new SleepAction(0.2)));
        hangUp();
        Actions.runBlocking(new ParallelAction(new SleepAction(0.2)));
        hangAlmostDown();
        Actions.runBlocking(new ParallelAction(new SleepAction(0.2)));
        hangUp();
    }
    public Action returnBMacro(double timeout, Gamepad gamepad) {
        return new bMacroTeleTimeout(timeout, gamepad1);
    }
    public class bMacroTeleTimeout implements Action {
        public double timeout;
        public ElapsedTime timer = new ElapsedTime();
        public Gamepad gamepad1;
        public bMacroTeleTimeout(double timeout, Gamepad gamepad1) {
            this.timeout = timeout;
            timer.reset();
            this.gamepad1 = gamepad1;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (gamepad1.b) {
                touchyRetract();
                grippyClose();
                flippy.setPosition(scaleFlippy(0.8));
                armTarget = 1600;
                Actions.runBlocking(new ParallelAction(new SleepAction(0.55), returnTelePid(0.55)));
                grippyOpen();
                Actions.runBlocking(new SleepAction(0.1));
//            flippy.setPosition(0.748);
//            armTarget = 0;
//            slideTarget = 0;
                flippy.setPosition(scaleFlippy(1));
                armTarget = 2540;
                slideTarget = 0;
                twisty.setPosition(scaleTwisty(0));
            }

            return timer.seconds() <= timeout;
        }
    }
    public void scoringMacro(Gamepad gamepad1, Gamepad gamepad2) {
        GamepadEx gamepad1Ex = new GamepadEx(gamepad1);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
//        if (gamepad1.right_bumper) {
//            newSpeciScoreAutomated();
//        }
//        if (gamepad1.left_bumper) {
//            newSpeciPickupAutomated();
//        }
        if (gamepad2.left_trigger > 0) {
            hangAlmostDown();
            Actions.runBlocking(new ParallelAction(new SleepAction(0.2), returnTelePid(0.2), returnTeleDriving(0.2, gamepad1)));
            hangUp();
            Actions.runBlocking(new ParallelAction(new SleepAction(0.2), returnTelePid(0.2), returnTeleDriving(0.2, gamepad1)));
            hangAlmostDown();
            Actions.runBlocking(new ParallelAction(new SleepAction(0.2), returnTelePid(0.2), returnTeleDriving(0.2, gamepad1)));
            hangUp();
            sweepyUp();
        }
        if (gamepad1.dpad_up) {
            twisty.setPosition(convertDegreesToTwisty(convertToClawDegree(getSampleAngle())));
        }
        if (gamepad1.right_trigger > 0) {
            flippy.setPosition(scaleFlippy(1));
            armTarget = 2490;
            slideTarget = 0;
            twisty.setPosition(scaleTwisty(0));
        }
        if (gamepad1.left_trigger > 0) {
            grippyOpen();
            Actions.runBlocking(new SleepAction(0.1));
            armTarget = 0;
            slideTarget = 0;
        }
        if (gamepad1.right_bumper) {
            armTarget -= 5;
        }
        else if (gamepad1.left_bumper) {
            armTarget += 5;
        }
        if (gamepad2.y && !(gamepad2.right_trigger > 0)) {
            touchyRetract();
            rightBumperCounter = 0;
            twisty.setPosition(scaleTwisty(0));
            flippy.setPosition(scaleFlippy(0.828));
            armTarget = 1940;

            while (Math.abs(armTarget - flip.getCurrentPosition()) > 1000) {
                TeleopPID(gamepad2);
                arcadeDrive(gamepad1);
                extendIntoSub(gamepad1, gamepad2);
            }
            slideTarget = 4600;
        }
        if (gamepad2.left_bumper && !(gamepad2.right_trigger > 0)) flippy.setPosition(scaleFlippy(0.97));
        if (gamepad2.right_bumper && !(gamepad2.right_trigger > 0)) flippy.setPosition(scaleFlippy(0.4));
        if (gamepad1.b) {
            touchyRetract();
            grippyOpen();
            Actions.runBlocking(new SleepAction(0.1));

            flippy.setPosition(scaleFlippy(1));
            armTarget = 1800;
            slideTarget = 0;
            Actions.runBlocking(new ParallelAction(new SleepAction(1), returnTeleDriving(1, gamepad1), returnTelePid(1)));
            flippy.setPosition(scaleFlippy(1));
            armTarget = 2537;
            slideTarget = 0;
            twisty.setPosition(scaleTwisty(0));
        }
        if (gamepad1.x) {
//            touchyTouch();
            grippyClose();

//            flippy.setPosition(0.718);
//            twisty.setPosition(1);
//            Actions.runBlocking(new SleepAction(0.1));
//            armTarget = 2120;
//            slideTarget = 1256;
            Actions.runBlocking(new SleepAction(0.3));
            flippy.setPosition(scaleFlippy(0.4));

            armTarget = 1250;
            slideTarget = 2700;
//            Actions.runBlocking(new ParallelAction(new SleepAction(1), returnTelePid(1), returnTeleDriving(1, gamepad1), returnBMacro(1, gamepad1)));
//            grippyClose();
        }
        else if (gamepad2.left_stick_button && !(gamepad2.right_trigger > 0)) {
            touchyRetract();
            flippy.setPosition(scaleFlippy(0.4));
            armTarget = 1900;
            slideTarget = 4600;
        }
        else if (gamepad2.right_stick_button && !(gamepad2.right_trigger > 0)) {
            touchyRetract();
            slideTarget = 962;
            Actions.runBlocking(new ParallelAction(new SleepAction(0.75), returnTelePid(0.75)));
            armTarget = 1300;
        }
        if (gamepad1Ex.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            twisty.setPosition(scaleTwisty(1));
        }
        else if (gamepad1Ex.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            twisty.setPosition(scaleTwisty(0.625));
        }

        if (gamepad2.a && !(gamepad2.right_trigger > 0)) {
            touchyRetract();
            rightBumperCounter = 0;
            flippy.setPosition(scaleFlippy(0.4));
            slideTarget = 0;
            intakeMultiplier = 1;
            while (Math.abs(slideTarget - slide.getCurrentPosition()) > 1000) {
                TeleopPID(gamepad2);
                arcadeDrive(gamepad1);
                extendIntoSub(gamepad1, gamepad2);
            }
//            wrist.setPosition(0.35);
            armTarget = 0;
            twisty.setPosition(scaleTwisty(0));
            grippy.setPosition(0);
        }
        if (gamepad2.x && !(gamepad2.right_trigger > 0)) {
            touchyRetract();
            rightBumperCounter = 0;
            slideTarget = 2300;
            armTarget = 280;
            intakeMultiplier = 1;
            flippy.setPosition(scaleFlippy(0.4));
            grippyOpen();

            twisty.setPosition(scaleTwisty(0));
            grippy.setPosition(0);
        }
        else if (gamepad2.b && !(gamepad2.right_trigger > 0)) {
            touchyRetract();
            rightBumperCounter = 0;
            armTarget = 0;
            intakeMultiplier = 1;

            while (Math.abs(armTarget - flip.getCurrentPosition()) > 50) {
                TeleopPID(gamepad2);
                arcadeDrive(gamepad1);
            }
            Actions.runBlocking(new SleepAction(0.05));

            grippy.setPosition(1);

            Actions.runBlocking(new SleepAction(0.5));

            flippy.setPosition(scaleFlippy(1));

            Actions.runBlocking(new SleepAction(0.25));

            twisty.setPosition(scaleTwisty(0));
            slideTarget = 0;
            while (Math.abs(slideTarget - slide.getCurrentPosition()) > 500) {
                TeleopPID(gamepad2);
                arcadeDrive(gamepad1);
            }
        }
        gamepad1Ex.readButtons();
        robotX = drive.pose.position.x;
        robotY = drive.pose.position.y;
        robotH = drive.pose.heading.toDouble();
    }
    /*
    public Action followPathInstant(Follower follower, PathChain path) {
        return new InstantAction(() -> follower.followPath(path, true));
    }

    public Action followPath(Follower follower, PathChain path) {
        return new followingPath(follower, path);
    }

    public class followingPath implements Action {
        Follower follower;
        PathChain path;

        public followingPath(Follower follower, PathChain path) {
            this.follower = follower;
            this.path = path;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            follower.followPath(path, true);
            return follower.isBusy();
        }
    }
    */
     
    public void clawControl(Gamepad gamepad) {
        if (gamepad.dpad_left && flipPos < 1800 && slidePos < 4400 && !(gamepad.left_trigger > 0)) grippy.setPosition(0);
        else if (gamepad.dpad_left && flipPos > 1800 && slidePos > 4400 && !(gamepad.left_trigger > 0)) {
            grippyOpen();
            Actions.runBlocking(new ParallelAction(new SleepAction(0.2), returnTeleDriving(0.2, gamepad1), returnTelePid(0.2)));
            flippy.setPosition(scaleFlippy(0.4));
        }
        else if (gamepad.dpad_left && slidePos < 1000 && !(gamepad.left_trigger > 0)) {
            grippyOpen();
        }

        else if (gamepad.dpad_right && !(gamepad.left_trigger > 0)) grippy.setPosition(1);
    }
    public void twistyControl(Gamepad gamepad) {
        if (gamepad.dpad_up && !(gamepad.left_trigger > 0)) twisty.setPosition(0);
        else if (gamepad.dpad_down && !(gamepad.left_trigger > 0)) twisty.setPosition(0.5);
    }
    public void hangControl(Gamepad gamepad) {
        if (gamepad.left_trigger > 0) {
            leftHang.setPosition(0);
            rightHang.setPosition(0);
            flippy.setPosition(scaleFlippy(1));
        }
        else if (gamepad.right_trigger > 0) {
            leftHang.setPosition(1);
            rightHang.setPosition(1);
            flippy.setPosition(scaleFlippy(0.4));
        }
    }
    public void TeleopPID(Gamepad gamepad) {
        armTarget += (int) ((int) -gamepad.right_stick_y * 50);
        slideTarget += (int) -gamepad.left_stick_y * 100;
//        int targetLength = (int) (1750*(1/Math.cos(Math.toRadians(flipPos/armPIDValues.ticks_in_degree))));
//        slideExtensionLimit = targetLength;

        if (armTarget < 0) armTarget = 0;
        else if (armTarget > 2550) armTarget = 2550;

        if (slideTarget < 0) slideTarget = 0;
//        else if (slideTarget > targetLength && flipPos < 1850) slideTarget = targetLength;
        else if (slideTarget > 4600 && armTarget > 1000) slideTarget = 4600;
        else if (slideTarget > 1830 && armTarget < 1000) slideTarget = 1830;

        flipPos = flip.getCurrentPosition();
        slidePos = slide.getCurrentPosition();

        double pid = armController.calculate(flipPos, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / armPIDValues.ticks_in_degree)) * armPIDValues.fF;

        double power = pid + ff;

        flip.setPower(power);

        double pid2 = slideController.calculate(slidePos, scaleSlides(slideTarget));

        slide.setPower(-pid2);

    }
    public void slidesPID(Gamepad gamepad) {
//        double ff = Math.cos(Math.toRadians(armTarget / armPIDValues.ticks_in_degree)) * armPIDValues.fF;
//        flip.setPower((-gamepad.right_stick_y * 0.25) + ff);

        slideTarget += (int) -gamepad.left_stick_y * 28;
        if (slideTarget < 0) slideTarget = 0;
        else if (slideTarget > 5000) slideTarget = 5000;
        slidePos = slide.getCurrentPosition();

        double pid2 = slideController.calculate(slidePos, slideTarget);

        slide.setPower(-pid2);
    }

    public void extraD1Features(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            slideTarget += 28;
            if (slideTarget > 5000) {slideTarget = 5000; }
        }
        else if (gamepad.dpad_down) {
            slideTarget -= 28;
        }
        else if (gamepad.dpad_right) {
            armTarget += 15;
        }
        else if (gamepad.dpad_left) {
            armTarget -= 15;
        }
//        else if (gamepad.right_bumper) {
//            leftHang.setPosition(leftHangPosUp);
//            rightHang.setPosition(rightHangPosUp);
//        }
//        else if(gamepad.left_bumper){
//            leftHang.setPosition(leftHangPosDown);
//            rightHang.setPosition(rightHangPosDown);
//        }

//        intakeRight.setPower((-gamepad.left_trigger + gamepad.right_trigger));
//        intakeLeft.setPower(gamepad.left_trigger - gamepad.right_trigger);

//        if (gamepad.left_trigger > 0 && !(gamepad.right_trigger > 0)) {
//            intakeLeft.setPower(1);
//            intakeRight.setPower(-1);
//        }
//        if (gamepad.right_trigger > 0 && !(gamepad.left_trigger > 0)) {
//            intakeLeft.setPower(-1);
//            intakeRight.setPower(1);
//        }

//        if (gamepad.y) {
//            wrist.setPosition(0.92);
//        }
//        else if (gamepad.left_bumper) {
//            wrist.setPosition(0);
//        }
//        else if (gamepad.right_bumper) {
//            wrist.setPosition(0.5);
//        }
    }

    public double scaleTwisty(double unscaled) {
//        return 0.35733*(unscaled*unscaled) + 0.312667*unscaled;
        return (-0.533333*(unscaled*unscaled) - 0.466667*unscaled + 1);
    }
    public double scaleFlippy(double unscaled) {
        return 1.81818*unscaled - 0.727272;
    }
    public int scaleSlides(double unscaled) {
        return ((int) (unscaled * 0.7137546468*0.7135416667 * 1.9635036496));
    }
    public Action pidToPoint(Pose2d targetPose) {
        return new p2p(this, targetPose);
    }
    public void runPidToPoint(p2p p2p, Pose2d targetPose) {
        p2p.updateTarget(targetPose);
        p2p.run();
    }
    public static class p2p implements Action {
        MecanumDrive drive;
        PIDCoefficients x, h;
        double lateralMultiplier;
        public PIDController xController, yController, hController;
        public Pose2d target;
        Robot bot;

        public p2p(Robot bot, Pose2d target) {
            this.bot = bot;
            drive = bot.drive;
            x = bot.x;
            h = bot.h;
            lateralMultiplier = bot.lateralMultiplier;

            xController = new PIDController(x.p, x.i, x.d);
            yController = new PIDController(x.p, x.i, x.d);
            hController = new PIDController(h.p, h.i, h.d);

            this.target = target;
        }
        public void updateTarget(Pose2d newTarget) {
            target = newTarget;
        }
        public void updatePids() {
            xController = new PIDController(bot.x.p, bot.x.i, bot.x.d);
            yController = new PIDController(bot.x.p, bot.x.i, bot.x.d);
            hController = new PIDController(bot.h.p, bot.h.i, bot.h.d);
        }
        public boolean closeEnough(double epsilon) {
            return ((Math.abs(drive.pose.position.x - target.position.x) < epsilon) && (Math.abs(drive.pose.position.y - target.position.y) < epsilon) && (Math.abs(drive.pose.heading.toDouble() - target.heading.toDouble()) < 0.5*epsilon));
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            updatePids();

            double xPower = xController.calculate(drive.pose.position.x, target.position.x);
            double yPower = lateralMultiplier * yController.calculate(drive.pose.position.y, target.position.y);
            double hPower = hController.calculate(drive.pose.heading.toDouble(), target.heading.toDouble());

            xPower = Range.clip(xPower, -1, 1);
            yPower = Range.clip(yPower, -1, 1);
            hPower = Range.clip(hPower, -1, 1);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(xPower, yPower), hPower));
            return !closeEnough(bot.epsilon);
        }
        public void run() {
            updatePids();

            double xPower = xController.calculate(drive.pose.position.x, target.position.x);
            double yPower = lateralMultiplier * yController.calculate(drive.pose.position.y, target.position.y);
            double hPower = hController.calculate(drive.pose.heading.toDouble(), target.heading.toDouble());

            xPower = Range.clip(xPower, -1, 1);
            yPower = Range.clip(yPower, -1, 1);
            hPower = Range.clip(hPower, -1, 1);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(xPower, yPower), hPower));
        }
    }
    public void startPID() {
        if (currentThread == null || !currentThread.isAlive()) {
            stopPid = false;
            currentThread = new Thread(new pidfLoopAuton());
            currentThread.start();
        }
    }
    public void setIntakePower(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(-power);
    }
    public Action setArmVals(int pivot, int slide) {
        return new InstantAction(() -> {armTarget = pivot; slideTarget = slide;});
    }
//    public Action setWrist(double pos) {
//        return new InstantAction(() -> wrist.setPosition(pos));
//    }
    public Action setIntake(double power) {
        return new InstantAction(() -> {intakeLeft.setPower(power); intakeRight.setPower(-power);});
    }

    public Action intake(double power) {
        return new intakeAction(power);
    }
    public class intakeAction implements Action {
        double intakePower;

        public intakeAction(double power) {
            intakePower = power;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setIntakePower(intakePower);
            return false;
        }
    }
    public void setAutoTargetToTELE() {
        armTarget = armTargetAuto;
        slideTarget = slideTargetAuto;
    }
    public Action setPidVals(int arm, int slide) {
        return new ValAction(arm, slide);
    }
    public Action wrist(double wrist) {
        return new wristAction(wrist);
    }
    public class wristAction implements Action {
        double wristPos;
        public wristAction(double wrist) {
            wristPos = wrist;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(wristPos);
            return false;
        }
    }
    public Action getPIDAction() {
        return new pidfLoopAction();
    }
    public void stopPidAction() {
        stopPid = true;
    }
    public Action pidfLoopSingular(int armTarget) {
        return new pidfLoopActionSingular(armTarget);
    }
    public class pidfLoopActionSingular implements Action {
        int armTarget;
        public pidfLoopActionSingular(int armTarget) {
            this.armTarget = armTarget;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            flipPos = flip.getCurrentPosition();
            slidePos = slide.getCurrentPosition();

            double pid = armController.calculate(flipPos, armTargetAuto);
            double ff = Math.cos(Math.toRadians(armTargetAuto / armPIDValues.ticks_in_degree)) * armPIDValues.fF;

            double power = pid + ff;

            flip.setPower(power);

            double pid2 = slideController.calculate(slidePos, scaleSlides(slideTargetAuto));

            slide.setPower(-pid2);

            if (Math.abs(flipPos - armTarget) <= 10) {
                flip.setPower(0);
            }

            return (Math.abs(flipPos - armTarget) > 10);
        }
    }
    public class pidfLoopAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            flipPos = flip.getCurrentPosition();
            slidePos = slide.getCurrentPosition();

            double pid = armController.calculate(flipPos, armTargetAuto);
            double ff = Math.cos(Math.toRadians(armTargetAuto / armPIDValues.ticks_in_degree)) * armPIDValues.fF;

            double power = pid + ff;

            flip.setPower(power);

            double pid2 = slideController.calculate(slidePos, scaleSlides(slideTargetAuto));

            slide.setPower(-pid2);

//            try {
//                Thread.sleep(10); // Adjust the sleep time as needed
//            } catch (InterruptedException e) {
//                Thread.currentThread().interrupt();
//            }
            return !stopPid;
        }
    }
    public Action returnTelePid(double timeout) {
        return new pidfLoopActionTeleTimeout(timeout);
    }
    public Action returnCancelableTelePID() {
        return new pidfLoopActionTeleCancelable();
    }
    public Action returnTeleDriving(double timeout, Gamepad gamepad1) {
        return new driveActionTeleTimeout(timeout, gamepad1);
    }
    public class driveActionTeleTimeout implements Action {
        public double timeout;
        public ElapsedTime timer = new ElapsedTime();
        public Gamepad gamepad1;
        public driveActionTeleTimeout(double timeout, Gamepad gamepad1) {
            this.timeout = timeout;
            timer.reset();
            this.gamepad1 = gamepad1;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -0.75*gamepad1.right_stick_x;

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-y,x), rx));

            return timer.seconds() <= timeout;
        }
    }
    public class pidfLoopActionTeleTimeout implements Action {
        public double timeout;
        public ElapsedTime timer = new ElapsedTime();
        public pidfLoopActionTeleTimeout(double timeout) {
            this.timeout = timeout;
            timer.reset();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            flipPos = flip.getCurrentPosition();
            slidePos = slide.getCurrentPosition();

            double pid = armController.calculate(flipPos, armTarget);
            double ff = Math.cos(Math.toRadians(armTargetAuto / armPIDValues.ticks_in_degree)) * armPIDValues.fF;

            double power = pid + ff;

            flip.setPower(power);

            double pid2 = slideController.calculate(slidePos, scaleSlides(slideTarget));

            slide.setPower(-pid2);

//            try {
//                Thread.sleep(10); // Adjust the sleep time as needed
//            } catch (InterruptedException e) {
//                Thread.currentThread().interrupt();
//            }
            return timer.seconds() <= timeout;
        }
    }
    public class pidfLoopActionTeleCancelable implements Action {
        public ElapsedTime timer = new ElapsedTime();

        public pidfLoopActionTeleCancelable() {
            timer.reset();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            flipPos = flip.getCurrentPosition();
            slidePos = slide.getCurrentPosition();

            double pid = armController.calculate(flipPos, armTarget);
            double ff = Math.cos(Math.toRadians(armTargetAuto / armPIDValues.ticks_in_degree)) * armPIDValues.fF;

            double power = pid + ff;

            flip.setPower(power);

            double pid2 = slideController.calculate(slidePos, scaleSlides(slideTarget));

            slide.setPower(-pid2);

            return !endPID;
        }
    }
    public class ValAction implements Action {
        int armTarget, slidetarget;
        public ValAction(int arm, int slide) {
            armTarget = arm;
            slidetarget = slide;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPidValues(armTarget, slidetarget);
            return false;
        }
    }
    private class pidfLoopAuton implements Runnable {
        @Override
        public void run() {
            while (!stopPid) {
                flipPos = flip.getCurrentPosition();
                slidePos = slide.getCurrentPosition();

                double pid = armController.calculate(flipPos, armTargetAuto);
                double ff = Math.cos(Math.toRadians(armTargetAuto / armPIDValues.ticks_in_degree)) * armPIDValues.fF;

                double power = pid + ff;

                flip.setPower(power * pivotMultiplier);

                double pid2 = slideController.calculate(slidePos, scaleSlides(slideTargetAuto));

                slide.setPower(-pid2);

                try {
                    Thread.sleep(10); // Adjust the sleep time as needed
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }
    public Action stopPID() {
        return new stopPid();
    }
    public class stopPid implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Robot.stopPid = true;
            try {
                if (currentThread != null && currentThread.isAlive()) {
                    currentThread.join(); // Wait for the thread to finish
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
           // currentThread.stop();

            armTargetAuto = 0;
            slideTargetAuto = 0;
            flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return false;
        }
    }
    public void setPidValues(int arm, int slide) {
        armTargetAuto = arm;
        slideTargetAuto = slide;
    }
    public void setPivotMultiplier(double multiplier) {
        pivotMultiplier = multiplier;
    }

    public static class armPIDValues {
        public static double fP = 0.0035, fI = 0, fD = 0;  //fD = 0.00001, fP = 0.002
        public static double fF = 0.008; //fF = 0.0022
        public static double sP = 0.005, sI, sD;

        public static double ticks_in_degree = 1850 / 90.0;
    }
    //4000, 2000

    //TODO: Limelight Actions Below
    public static class searchForSample implements Action {
        double timeout;
        public boolean finished = false;
        Robot bot;


        public searchForSample(Robot bot, double timeout) {
            this.bot = bot;

            this.timeout = timeout;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bot.limelight.pipelineSwitch(0);
            bot.limelight.start();

            LLResult result = bot.limelight.getLatestResult();



            //TODO: add rest of code later

            return !finished;
        }
    }

    public static double convertXPixelsToCamInches(double pixels) {
        return xA * (1/Math.cos(xB * pixels + xC)) + xD;
    }
    public static double convertYpixelsToCamInches(double pixels) {
        return yA * (1/Math.cos(yB * pixels + yC)) + yD;
    }
    public static Vector2d getDistFromCamera(double pixelsX, double pixelsY) {
        return new Vector2d(convertXPixelsToCamInches(pixelsX), convertYpixelsToCamInches(pixelsY));
    }

    //TODO: AUTO ACTIONS BELOW

    public Action pid(ArmPosition target) {
        return new PidAction(target);
    }
    public class PidAction implements Action {
        int flipTarget, slideTarget;
        double grippy, flippy, twisty, touchy;


        public PidAction(ArmPosition target) {
            this.flipTarget = target.pivot;
            this.slideTarget = target.slides;

            this.grippy = target.grippy;
            this.flippy = target.flippy;
            this.twisty = target.twisty;
            this.touchy = target.touchy;

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Robot.this.grippy.setPosition(grippy);
            Robot.this.flippy.setPosition(flippy);
            Robot.this.twisty.setPosition(twisty);
            Robot.this.touchy.setPosition(touchy);

            setPidValues(flipTarget, slideTarget);

            return (((Math.abs(flipPos - flipTarget) < 20) && (Math.abs(slidePos - slideTarget) < 50)));
        }
    }

    public class ArmPosition {
        int pivot, slides;
        double grippy, flippy, twisty, touchy;
        public ArmPosition(int pivot, int slides, double grippy, double flippy, double twisty, double touchy) {
            this.pivot = pivot;
            this.slides = slides;

            this.grippy = grippy;
            this.flippy = flippy;
            this.twisty = twisty;
            this.touchy = touchy;
        }
        public ArmPosition() {
            pivot = -1;
            slides = -1;
        }
    }
    public Action autoAction(Action driveAction, ArmPosition target) {
        return new AutoAction(driveAction, target);
    }

    public class AutoAction implements Action {
        Action driveAction;
        ArmPosition target;

        public AutoAction(Action driveAction, ArmPosition target) {
            this.driveAction = driveAction;
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Actions.runBlocking(new ParallelAction(driveAction, pid(target)));
            return false;
        }
    }
    public static class HeadingPIDController {
        PIDCoefficients coeffs;
        double lastError = 0;
        double integralSum = 0;

        public HeadingPIDController(PIDCoefficients coeffs) {
            this.coeffs = coeffs;
        }

        public void setPID(PIDCoefficients coeffs) {
            this.coeffs = coeffs;
        }

        public double calculate(double current, double target) {
            double currentTimeStamp = (double) System.nanoTime() / 1E9;

            double error = AngleUnit.normalizeRadians(target - current);

            double derivative = (error - lastError) / currentTimeStamp;

            // sum of all error over time
            integralSum = integralSum + (error * currentTimeStamp);

            double out = (coeffs.p * error) + (coeffs.i * integralSum) + (coeffs.d * derivative);

            lastError = error;

            return out;
        }

    }
}

