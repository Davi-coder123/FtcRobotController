package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.RobotHardware;

@TeleOp
public class GarraMotorExemplo extends LinearOpMode {

    RobotHardware robot       = new RobotHardware(this);

    public void runOpMode() {

        double drive;
        double turn;
        double garraOffset   = 0;
        double braco;

        //Inicializa todos os hardwares, com o HardwareClass
        robot.init();

        waitForStart();

        while (opModeIsActive()) {

            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            robot.driveRobot(drive, turn);

            //Uso das constantes da classe robot
            if (gamepad1.right_bumper)
                garraOffset += robot.GARRA_SPEED;
            else if (gamepad1.left_bumper)
                garraOffset -= robot.GARRA_SPEED;
            garraOffset = Range.clip(garraOffset, -0.5, 0.5);

            //uso da função da classe robot.
            if (gamepad1.right_bumper){
                robot.setGarraPositions(garraOffset);
            }

            //Uso de constantes de motor
            if (gamepad1.y)
                braco = robot.BRACO_UP_POWER;
            else if (gamepad1.a)
                braco = robot.BRACO_UP_POWER;
            else
                braco = 0;

            robot.setArmPower(braco);

            // Manda telemetria para explicar como fazer/valores
            telemetry.addData("braco Up/Down", "Y & A Buttons");
            telemetry.addData("Hand Open/Closed", "Left and Right Bumpers");
            telemetry.addData("-", "-------");

            telemetry.addData("braco Power",  "%.2f", braco);
            telemetry.addData("Hand Position",  "Offset = %.2f", garraOffset);
            telemetry.update();

            //EXTRA
            //todo: isso é um "To-do" (para fazer)
        }
    }


}