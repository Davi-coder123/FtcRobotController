package org.firstinspires.ftc.teamcode.drive.camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drive.actuators.Intake1;
import org.firstinspires.ftc.teamcode.drive.actuators.Outtake1;

@Autonomous(name="Auto Blue Side (Limelight)", group="Autonomous")
public class AutoBlueSideLimelight extends LinearOpMode {
    Outtake1 outtake;
    Intake1 intake;

    public Limelight3A limelight;
    public IMU imu;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Posição alvo da base (em cm e graus)
    static final double TARGET_X = 192;
    static final double TARGET_Y = 163;
    static final double TARGET_HEADING = 0;

    // Ganhos de controle
    static final double KP_X = 0.025;
    static final double KP_Y = 0.025;
    static final double KP_HEADING = 0.01;

    // Tolerâncias
    static final double POS_TOLERANCE = 1.0; // cm
    static final double ANG_TOLERANCE = 2.0; // graus

    boolean reachedBase = false;

    DcMotor poliaright;
    DcMotor polialeft;
    Servo Bright;
    Servo Bleft;
    Servo garrinha;
    double ticks = 2750;
    double newTarget;
    Servo rotate;
    Servo garra;
    Servo pleft;
    Servo pright;
    Servo lright;
    Servo lleft;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        outtake = new Outtake1(hardwareMap);
        intake = new Intake1(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(myIMUparameters);

        lright = hardwareMap.get(Servo.class, "lright");
        lleft = hardwareMap.get(Servo.class, "lleft");
        garra = hardwareMap.get(Servo.class, "garra");
        pleft = hardwareMap.get(Servo.class, "pleft");
        pright = hardwareMap.get(Servo.class, "pright");
        rotate = hardwareMap.get(Servo.class, "rotate");

        poliaright = hardwareMap.get(DcMotor.class, "poliaright");
        polialeft = hardwareMap.get(DcMotor.class, "polialeft");
        Bright = hardwareMap.get(Servo.class, "bright");
        Bleft = hardwareMap.get(Servo.class, "bleft");
        garrinha = hardwareMap.get(Servo.class, "garrinha");

        poliaright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        polialeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        polialeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            rotate.setPosition(0.7);
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                limelight.updateRobotOrientation(robotYaw);
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x * 100;
                    double y = botpose_mt2.getPosition().y * 100;
                    double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

                    double errorX = TARGET_X - x;
                    double errorY = TARGET_Y - y;
                    double errorHeading = angleDiff(TARGET_HEADING, heading);

                    telemetry.addData("Pose (cm,°)", "X: %.1f  Y: %.1f  H: %.1f", x, y, heading);
                    telemetry.addData("Erros", "dX: %.1f  dY: %.1f  dH: %.1f", errorX, errorY, errorHeading);

                    if (!reachedBase) {
                        if (Math.abs(errorX) < POS_TOLERANCE &&
                                Math.abs(errorY) < POS_TOLERANCE &&
                                Math.abs(errorHeading) < ANG_TOLERANCE) {
                            stopAllMotors();
                            reachedBase = true;
                        } else {
                            // Correção proporcional
                            double drive = KP_X * errorX;
                            double strafe = KP_Y * errorY;
                            double turn = KP_HEADING * errorHeading;

                            // Limitar potências
                            drive = Math.max(-0.3, Math.min(0.3, drive));
                            strafe = Math.max(-0.3, Math.min(0.3, strafe));
                            turn = Math.max(-0.25, Math.min(0.25, turn));

                            mecanumDrive(drive, strafe, turn);
                        }
                    } else {
                        // 🚀 Sequência após alinhar na base
                        SlidesUP();
                        sleep(500);
                        Entrega();
                        IntakeFORWARD();
                        sleep(500);
                        IntakeBACKWARD();
                        sleep(500);
                        SlidesDOWN();
                        break; // Encerra autonomous
                    }
                }
            } else {
                telemetry.addData("Limelight", "No Targets");
            }
            telemetry.update();
        }
    }

    // -------- Funções de movimento --------
    private void mecanumDrive(double drive, double strafe, double turn) {
        double fl = drive - strafe - turn;
        double bl = drive + strafe - turn;
        double fr = drive + strafe + turn;
        double br = drive - strafe + turn;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    private void stopAllMotors() {
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setPower(0);
        }
    }

    private double angleDiff(double target, double current) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    // -------- Slides e mecanismos --------
    public void SlidesUP(){
        newTarget = ticks;
        poliaright.setTargetPosition((int) newTarget);
        polialeft.setTargetPosition((int) newTarget);
        poliaright.setPower(1);
        polialeft.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void SlidesDOWN(){
        poliaright.setTargetPosition(0);
        polialeft.setTargetPosition(0);
        poliaright.setPower(1);
        polialeft.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bright.setPosition(1);
        Bleft.setPosition(0);
        sleep(1000);
        polialeft.setPower(0);
        poliaright.setPower(0);
    }
    public void Entrega(){
        Bright.setPosition(0.4);
        Bleft.setPosition(0.6);
        sleep(1000);
        Bright.setPosition(1);
        Bleft.setPosition(0);
    }
    public void IntakeFORWARD(){
        lright.setPosition(0.6);
        lleft.setPosition(0.7);
        sleep(500);
        garra.setPosition(0.3);
        pleft.setPosition(0);
        pright.setPosition(1);
    }
    public void IntakeBACKWARD(){
        garra.setPosition(0.6);
        sleep(200);
        pleft.setPosition(0.8);
        pright.setPosition(0.2);
        lright.setPosition(1);
        lleft.setPosition(0.1);
        sleep(500);
        garra.setPosition(0.3);
        sleep(500);
    }
}
