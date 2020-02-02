package org.firstinspires.ftc.teamcode.relicrecovery_2018.team11364_diamondplate;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Relic Recovery 11364 A", group="2018")
public class TeleOpA extends OpMode {
    Robot robot;
    double pFactor;
    double grabberPosition = 0.183;
    double spoolPower;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.servo_jewel2.setPosition(0.45);
        robot.servo_jewel1.setPosition(0.34);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) pFactor = 0.4;
        else pFactor = 1.0;

        if (gamepad1.x) robot.motor_grab.setPower(-0.5);
        else if (gamepad1.b) robot.motor_grab.setPower(0.5);
        else robot.motor_grab.setPower(0.0);

        double x = gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double z = -gamepad1.right_stick_x;

        robot.motor_frontLeft.setPower(stdRangeClipAndBuffer(x + y + z));
        robot.motor_backLeft.setPower(stdRangeClipAndBuffer(x - y + z));
        robot.motor_frontRight.setPower(stdRangeClipAndBuffer(-x + y + z));
        robot.motor_backRight.setPower(stdRangeClipAndBuffer(-x - y + z));

        if (gamepad1.left_trigger>0.03) spoolPower = -gamepad1.left_trigger;
        else if (gamepad1.right_trigger>0.03) spoolPower = gamepad1.right_trigger;
        else spoolPower = 0;

        robot.motor_spool.setPower(spoolPower);

        telemetry.addData("x", x);
        telemetry.addData("y",y);
        telemetry.addData("z", z);
        telemetry.addData("Spool Power", spoolPower);
        telemetry.addData("Grabber Position", grabberPosition);
        telemetry.update();
    }

    private double stdRangeClipAndBuffer(double input) {
        if (input > 1) return 1;
        else if (input < -1) return -1;
        else if (input < 0.05 & input > 0) return 0;
        else if (input > -0.05 & input < 0) return 0;
        else return input;
    }
}


