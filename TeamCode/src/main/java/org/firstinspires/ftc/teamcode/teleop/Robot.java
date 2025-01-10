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
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.R;

import java.util.Arrays;
import java.util.List;

@Config
//TODO: change claw opened and closed values
public class Robot {
//    public DcMotor leftFront, leftBack, rightFront, rightBack;
    public DcMotor flip, slide;
    public Servo wrist, leftHang, rightHang;
    public CRServo intakeLeft, intakeRight;
    public Servo grippy, twisty, flippy;
    public MecanumDrive drive;
    public PIDController armController, slideController;
    public double epsilon = 0.1;

    public PIDCoefficients x = new PIDCoefficients(0,0,0);
    public PIDCoefficients h = new PIDCoefficients(0,0,0);
    public double lateralMultiplier = 1;
    public int rightBumperCounter = 0;


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

    public Robot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

//        leftFront = hardwareMap.dcMotor.get("leftFront");
//        leftBack = hardwareMap.dcMotor.get("leftBack");
//        rightFront = hardwareMap.dcMotor.get("rightFront");
//        rightBack = hardwareMap.dcMotor.get("rightBack");

        flip = hardwareMap.dcMotor.get("flip");
        slide = hardwareMap.dcMotor.get("slide");

        grippy = hardwareMap.servo.get("claw");
        twisty = hardwareMap.servo.get("twist");
        flippy = hardwareMap.servo.get("flippy");

        leftHang = hardwareMap.servo.get("leftHang");
        rightHang = hardwareMap.servo.get("rightHang");

//        List<DcMotor> motors = Arrays.asList(leftBack, leftFront, rightBack, rightFront, flip, slide);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        flip.setDirection(DcMotorSimple.Direction.FORWARD);
        flip.setDirection(DcMotorSimple.Direction.FORWARD);

        slide.setDirection(DcMotorSimple.Direction.REVERSE);

//        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
        rightHang.setDirection(Servo.Direction.REVERSE);
        leftHang.setDirection(Servo.Direction.FORWARD);


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


    public void grippyClose() {
        grippy.setPosition(1);
    }
    public void grippyOpen() {
        grippy.setPosition(0);
    }
    public void speciMacro() {
        flippy.setPosition(0.1);
        twisty.setPosition(1);
        Actions.runBlocking(setPidVals(755, 2884));
    }
    public void speciPickup() {
        flippy.setPosition(0);
        Actions.runBlocking(setPidVals(2250,0));
        twisty.setPosition(0);
        grippy.setPosition(0);
    }
    public void reset() {
        intakeMultiplier = 1;
        Actions.runBlocking(setPidVals(0,0));
        flippy.setPosition(0.95);
        twisty.setPosition(0);
        grippy.setPosition(0);
    }
    public void scoringMacro(Gamepad gamepad1, Gamepad gamepad2) {
        GamepadEx gamepad1Ex = new GamepadEx(gamepad1);
        if (gamepad2.y) {
            rightBumperCounter = 0;
            flippy.setPosition(0.53);
            armTarget = 1870;

            while (Math.abs(armTarget - flip.getCurrentPosition()) > 1000) {
                TeleopPID(gamepad2);
                arcadeDrive(gamepad1);
            }
            slideTarget = 4600;
        }
        if (gamepad2.left_bumper) flippy.setPosition(0.95);
        if (gamepad2.right_bumper) flippy.setPosition(0);
        if (gamepad1.b) {
            rightBumperCounter = 0;
            grippyOpen();
            flippy.setPosition(0);
            armTarget = 2250;
            slideTarget = 0;
            twisty.setPosition(0);
        }
        if (gamepad1.x) {
            rightBumperCounter = 0;
            grippy.close();
            flippy.setPosition(0);
            twisty.setPosition(1);
            armTarget = 758;
            slideTarget = 2856;
        }
        if (gamepad1Ex.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            twisty.setPosition(1);
        }
        else if (gamepad1Ex.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            twisty.setPosition(0.625);
        }
        

        if (gamepad2.a) {
            rightBumperCounter = 0;
            flippy.setPosition(0.95);
            slideTarget = 0;
            intakeMultiplier = 1;
            while (Math.abs(slideTarget - slide.getCurrentPosition()) > 1000) {
                TeleopPID(gamepad2);
                arcadeDrive(gamepad1);
            }
//            wrist.setPosition(0.35);
            armTarget = 0;
            twisty.setPosition(0);
            grippy.setPosition(0);
        }
        if (gamepad2.x) {
            rightBumperCounter = 0;
            slideTarget = 2300;
            armTarget = 260;
            intakeMultiplier = 1;
            flippy.setPosition(1);
            while (Math.abs(slideTarget - slide.getCurrentPosition()) > 500) {
                TeleopPID(gamepad2);
                arcadeDrive(gamepad1);
            }

            twisty.setPosition(0);
            grippy.setPosition(0);
        }
        else if (gamepad2.b) {
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

            flippy.setPosition(0);

            Actions.runBlocking(new SleepAction(0.25));

            twisty.setPosition(0);
            slideTarget = 0;
            while (Math.abs(slideTarget - slide.getCurrentPosition()) > 500) {
                TeleopPID(gamepad2);
                arcadeDrive(gamepad1);
            }
        }
        gamepad1Ex.readButtons();
    }
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
    public void clawControl(Gamepad gamepad) {
        if (gamepad.dpad_left) grippy.setPosition(0);

        else if (gamepad.dpad_right) grippy.setPosition(1);
    }
    public void twistyControl(Gamepad gamepad) {
        if (gamepad.dpad_up) twisty.setPosition(1);
        else if (gamepad.dpad_down) twisty.setPosition(0.625);
    }
    public void hangControl(Gamepad gamepad) {
        if (gamepad.left_trigger > 0) {
            leftHang.setPosition(0);
            rightHang.setPosition(0);
            flippy.setPosition(1);
        }
        else if (gamepad.right_trigger > 0) {
            leftHang.setPosition(1);
            rightHang.setPosition(1);
            flippy.setPosition(0);
        }
    }
    public void TeleopPID(Gamepad gamepad) {
        armTarget += (int) ((int) -gamepad.right_stick_y * 30);
        slideTarget += (int) -gamepad.left_stick_y * 28;
//        int targetLength = (int) (1750*(1/Math.cos(Math.toRadians(flipPos/armPIDValues.ticks_in_degree))));
//        slideExtensionLimit = targetLength;

        if (armTarget < 0) armTarget = 0;
        else if (armTarget > 2300) armTarget = 2300;

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

        double pid2 = slideController.calculate(slidePos, slideTarget);

        slide.setPower(pid2);

    }
    public void slidesPID(Gamepad gamepad) {
//        double ff = Math.cos(Math.toRadians(armTarget / armPIDValues.ticks_in_degree)) * armPIDValues.fF;
//        flip.setPower((-gamepad.right_stick_y * 0.25) + ff);

        slideTarget += (int) -gamepad.left_stick_y * 28;
        if (slideTarget < 0) slideTarget = 0;
        else if (slideTarget > 5000) slideTarget = 5000;
        slidePos = slide.getCurrentPosition();

        double pid2 = slideController.calculate(slidePos, slideTarget);

        slide.setPower(pid2);
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
            Thread thread = new Thread(new pidfLoopAuton());
            currentThread = thread;
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

            double pid2 = slideController.calculate(slidePos, slideTargetAuto);

            slide.setPower(pid2);

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

            double pid2 = slideController.calculate(slidePos, slideTargetAuto);

            slide.setPower(pid2);

//            try {
//                Thread.sleep(10); // Adjust the sleep time as needed
//            } catch (InterruptedException e) {
//                Thread.currentThread().interrupt();
//            }
            return !stopPid;
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

                flip.setPower(power);

                double pid2 = slideController.calculate(slidePos, slideTargetAuto);

                slide.setPower(pid2);

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

    public static class armPIDValues {
        public static double fP = 0.01, fI = 0, fD = 0;  //fD = 0.00001, fP = 0.002
        public static double fF = 0.01; //fF = 0.0022
        public static double sP = 0.005, sI, sD;

        private static final double ticks_in_degree = 1850 / 90.0;
    }
    //4000, 2000
}

