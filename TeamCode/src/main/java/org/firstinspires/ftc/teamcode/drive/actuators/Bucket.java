package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket{
    Servo right;
    Servo left;

//construtor (ira pegar o hardware class de Teleop.java)
    public Bucket(HardwareMap hardwareMap){
        right = hardwareMap.get(Servo.class,"dir");
        left = hardwareMap.get(Servo.class,"esq");
    }

    public void turn(double positionRight, double positionLeft){
        right.setPosition(positionRight);
        left.setPosition(positionLeft);
    }
}
