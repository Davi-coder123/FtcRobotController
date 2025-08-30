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

/**
 * Classe de rotina autônoma para o lado VERMELHO usando visão por Limelight.
 * Controle PD (Proporcional + Derivativo) para drive, strafe e heading.
 *
 * - Detecta pose (x, y, heading) com Limelight e IMU.
 * - Move o robô até a base usando PD.
 * - Ao chegar na base, executa a sequência de mecanismos.
 */
@Autonomous(name="Auto Red Side (Limelight PD)", group="Autonomous")
public class AutoBlueSideLimelight extends LinearOpMode {
    // Subsistemas do robô
    Outtake1 outtake;
    Intake1 intake;

    // Sensores e hardware principais
    public Limelight3A limelight;
    public IMU imu;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ----------------- PARÂMETROS DE CONTROLE -----------------
    static  final double REAL_TARGET_X = 185;
    static final double TARGET_X = 185;     // cm
    static final double REAL_TARGET_Y = 160;
    static final double TARGET_Y = 163;     // cm
    static final double TARGET_HEADING = 0; // graus
    static final double TICKS_PER_CM = 17.82; // Para o nosso chassi

    // Ganhos de controle PD (P + Derivativo)
    static final double KP_X = 0.055, KD_X = 0.005;
    static final double KP_Y = 0.055, KD_Y = 0.005;
    static final double KP_HEADING = 0.01, KD_HEADING = 0.005;

    // Tolerâncias de chegada
    static final double POS_TOLERANCE = 2.0; // cm
    static final double ANG_TOLERANCE = 2.0; // graus
    static final double TURN_TOLERANCE = 1.5; // graus de margem de erro

    boolean reachedBase = false;

    // Histórico para cálculo derivado
    private double lastErrorX = 0, lastErrorY = 0, lastErrorHeading = 0;
    private long lastTime = 0;

    // ----------------- HARDWARE ADICIONAL -----------------
    DcMotor poliaright, polialeft;
    Servo Bright, Bleft, garrinha, rotate, garra, pleft, pright, lright, lleft;
    double ticks = 2750;
    double newTarget;

    @Override
    public void runOpMode() {
        // ----------------- INICIALIZAÇÃO -----------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        outtake = new Outtake1(hardwareMap);
        intake = new Intake1(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        // Servos
        lright = hardwareMap.get(Servo.class, "lright");
        lleft = hardwareMap.get(Servo.class, "lleft");
        garra = hardwareMap.get(Servo.class, "garra");
        pleft = hardwareMap.get(Servo.class, "pleft");
        pright = hardwareMap.get(Servo.class, "pright");
        rotate = hardwareMap.get(Servo.class, "rotate");
        Bright = hardwareMap.get(Servo.class, "bright");
        Bleft = hardwareMap.get(Servo.class, "bleft");
        garrinha = hardwareMap.get(Servo.class, "garrinha");

        // Slides
        poliaright = hardwareMap.get(DcMotor.class, "poliaright");
        polialeft = hardwareMap.get(DcMotor.class, "polialeft");
        poliaright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        polialeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        polialeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Drive mecanum
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        lastTime = System.currentTimeMillis();

        // ----------------- LOOP PRINCIPAL -----------------
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

                    // Erros
                    double errorX = REAL_TARGET_X - x;
                    double reachedErrorX = TARGET_X - x;
                    double errorY = REAL_TARGET_Y - y;
                    double reachedErrorY = TARGET_Y - y;
                    double errorHeading = angleDiff(TARGET_HEADING, heading);

                    long currentTime = System.currentTimeMillis();
                    double dt = (currentTime - lastTime) / 1000.0;

                    // Derivadas
                    double dX = dt > 0 ? (errorX - lastErrorX) / dt : 0;
                    double dY = dt > 0 ? (errorY - lastErrorY) / dt : 0;
                    double dH = dt > 0 ? (errorHeading - lastErrorHeading) / dt : 0;

                    // Controle PD
                    double drive = (KP_X * errorX) + (KD_X * dX);
                    double strafe = (KP_Y * errorY) + (KD_Y * dY);
                    double turn = (KP_HEADING * errorHeading) + (KD_HEADING * dH);

                    // Limite de potência
                    drive = Math.max(-0.3, Math.min(0.3, drive));
                    strafe = Math.max(-0.45, Math.min(0.45, strafe));
                    turn = Math.max(-0.25, Math.min(0.25, turn));

                    if (!reachedBase) {
                        if (Math.abs(reachedErrorX) < POS_TOLERANCE &&
                                Math.abs(reachedErrorY) < POS_TOLERANCE &&
                                Math.abs(errorHeading) < ANG_TOLERANCE) {
                            stopAllMotors();
                            reachedBase = true;
                        } else {
                            mecanumDrive(drive, strafe, turn);
                        }
                    }
                    else{
                        //Adicionar sequencia de passos aqui
                        //Depois adicionar uma verificação que pula direto para esse escopo se o tempo passou de "x" segundos para caso ele não se alinhe perfeitamente
                        telemetry.addData("Posição X: ", errorX);
                        telemetry.addData("Posição Y:", errorY);
                        telemetry.addData("Chegou na posição: ", reachedBase);
                        sleep(500);
                        strafeCM(13,0.15);
                        sleep(200);
                        SlidesUP();
                        moveCM(10.5,0.15);
                        sleep(100);
                        turnToAngle(-30);
                        Entrega();
                        IntakeFORWARD();
                        turnToAngle(-22);
                        sleep(100);
                        moveCM(-8,0.2);
                        SlidesDOWN();
                        sleep(400);
                        IntakeBACKWARD();
                        sleep(500);
                        SlidesUP();
                        moveCM(9, 0.15);
                        sleep(100);
                        turnToAngle(-30);
                        Entrega();
                        IntakeFORWARD();
                        sleep(100);
                        turnToAngle(-3);
                        sleep(100);
                        moveCM(-6, 0.2);
                        SlidesDOWN();
                        sleep(400);
                        IntakeBACKWARD();
                        sleep(500);
                        SlidesUP();
                        moveCM(5.5,0.15);
                        sleep(100);
                        turnToAngle(-30);
                        Entrega();
                        moveCM(-4,0.2);
                        SlidesDOWN();
                        break;

                    }

                    // Atualiza histórico
                    lastErrorX = errorX;
                    lastErrorY = errorY;
                    lastErrorHeading = errorHeading;
                    lastTime = currentTime;

                    // Debug
                    telemetry.addData("Pose", "X: %.1f Y: %.1f H: %.1f", x, y, heading);
                    telemetry.addData("Erro", "dX: %.1f dY: %.1f dH: %.1f", errorX, errorY, errorHeading);
                    telemetry.addData("Correção", "Drive: %.2f Strafe: %.2f Turn: %.2f", drive, strafe, turn);
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
        sleep(1100);
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

    private void strafeCM(double cm, double power) {
        int ticks = (int) (cm * TICKS_PER_CM);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - ticks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addLine("Strafando...");
            telemetry.update();
        }

        stopAllMotors();
    }
    private void moveCM(double cm, double power) {
        int ticks = (int) (cm * TICKS_PER_CM);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + ticks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addLine("Movendo para frente...");
            telemetry.update();
        }
        stopAllMotors();
    }
    private void turnToAngle(double targetAngle) {
        double currentYaw = getYaw();
        double error = angleDiff(targetAngle, currentYaw);

        // Corrigir modo dos motores
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        while (opModeIsActive() && Math.abs(error) > TURN_TOLERANCE) {
            double power = 0.25 * Math.signum(error);

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            currentYaw = getYaw();
            error = angleDiff(targetAngle, currentYaw);

            telemetry.addData("Yaw", currentYaw);
            telemetry.addData("Target", targetAngle);
            telemetry.update();

        }
        stopAllMotors();
    }
    private double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}