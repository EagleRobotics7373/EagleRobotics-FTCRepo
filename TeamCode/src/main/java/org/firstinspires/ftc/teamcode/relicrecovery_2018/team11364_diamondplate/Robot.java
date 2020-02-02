package org.firstinspires.ftc.teamcode.relicrecovery_2018.team11364_diamondplate;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    DcMotor motor_frontRight;
    DcMotor motor_frontLeft;
    DcMotor motor_backRight;
    DcMotor motor_backLeft;

    DcMotor motor_spool;
    DcMotor motor_grab;

    Servo servo_jewel1;
    Servo servo_jewel2;

    public Robot(HardwareMap hardwareMap) {
        motor_frontRight = hardwareMap.dcMotor.get("Motor_frontright");
        motor_frontLeft = hardwareMap.dcMotor.get("Motor_frontleft");
        motor_backRight = hardwareMap.dcMotor.get("Motor_backright");
        motor_backLeft = hardwareMap.dcMotor.get("Motor_backleft");

        motor_spool = hardwareMap.dcMotor.get("Motor_spool");
        motor_grab = hardwareMap.dcMotor.get("Motor_Grab");

        servo_jewel1 = hardwareMap.servo.get("Servo_jewel1");
        servo_jewel2 = hardwareMap.servo.get("Servo_jewel2");
    }
}
