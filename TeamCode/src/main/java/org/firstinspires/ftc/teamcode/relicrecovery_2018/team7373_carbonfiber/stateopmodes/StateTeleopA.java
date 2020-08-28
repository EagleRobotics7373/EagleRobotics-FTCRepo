/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.relicrecovery_2018.team7373_carbonfiber.stateopmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.relicrecovery_2018.team7373_carbonfiber.eaglerobotics.library.drivetrain.Holonomic;
import org.firstinspires.ftc.teamcode.relicrecovery_2018.team7373_carbonfiber.eaglerobotics.library.functions.MathOperations;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Relic Recovery 7373 TeleOp A", group = "2018")
//@Disabled
public class StateTeleopA extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();

  // Holonomic System
  DcMotor leftFrontMotor;
  DcMotor leftRearMotor;
  DcMotor rightFrontMotor;
  DcMotor rightRearMotor;

  Holonomic holonomic;

  double k = 1;

  // Lift and Grabber
  DcMotor lift;
//  DcMotor spinner;
  double lastPower = -.15;
//  Servo leftTop;
//  Servo leftBottom;
//  Servo rightTop;
//  Servo rightBottom;

  // Jewel Manipulator
//  Servo jewelManipulator;
//  Servo jewelRotator;

  Servo leftGrabber;
  Servo rightGrabber;

  // Relic Arm System
//  DcMotor threadedRodLift;
//  Servo relicGrabber;
//  Servo slideStop;
//
//  ColorSensor colorSensorLeft;
//  ColorSensor colorSensorRight;

    BNO055IMU imu;

    GlobalVarsState vars = new GlobalVarsState();

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    // Get motors from map
    leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
    leftRearMotor = hardwareMap.dcMotor.get("leftRearMotor");
    rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
    rightRearMotor = hardwareMap.dcMotor.get("rightRearMotor");

    holonomic = new Holonomic(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);


    lift = hardwareMap.dcMotor.get("lift");

    leftGrabber = hardwareMap.servo.get("leftGrabber");
    rightGrabber = hardwareMap.servo.get("rightGrabber");

//      BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//      parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//      parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//      parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//      parameters.loggingEnabled      = true;
//      parameters.loggingTag          = "IMU";
//      parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//      imu = hardwareMap.get(BNO055IMU.class, "imu");
//      imu.initialize(parameters);
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {

  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    runtime.reset();

  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());

    if (gamepad1.left_bumper) {
      lift.setPower(gamepad1.left_stick_y);
      holonomic.run(0, 0, 0);
    } else {
//      lift.setPower(0);
      // Run using cubic and Y reversed
      holonomic.run(MathOperations.pow(-k * gamepad1.left_stick_y, 3), MathOperations.pow(k * gamepad1.left_stick_x, 3),
              MathOperations.pow(k * gamepad1.right_stick_x, 3));
      holonomic.run(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

      if (gamepad1.left_trigger > 0.03) lift.setPower(-gamepad1.left_trigger);
      else if (gamepad1.right_trigger > 0.03) lift.setPower(gamepad1.right_trigger);
      else lift.setPower(0);
    }
//    if(gamepad1.dpad_left){
//      spinner.setPower(.5);
//      lastPower = .2;
//    } else if(gamepad1.dpad_right){
//      spinner.setPower(-.5);
//      lastPower = -.2;
//    } else {
//      spinner.setPower(lastPower);
//  }
    if (gamepad1.x) {
//      rightTop.setPosition(vars.rightTopClosedNew);
//      leftTop.setPosition(vars.leftTopClosedNew);
//      rightBottom.setPosition(vars.rightBottomClosedNew);
//      leftBottom.setPosition(vars.leftBottomClosedNew);
      rightGrabber.setPosition(vars.rightClosed);
      leftGrabber.setPosition(vars.leftClosed);


    } else if (gamepad1.y) {
//      rightTop.setPosition(vars.rightTopOpen);
//      leftTop.setPosition(vars.leftTopOpen);
//      rightBottom.setPosition(vars.rightBottomOpen);
//      leftBottom.setPosition(vars.leftBottomOpen);
      rightGrabber.setPosition(vars.rightOpen);
      leftGrabber.setPosition(vars.leftOpen);
    }

    // Run the Intake
//    spinner.setPower(lastPower);

//    jewelManipulator.setPosition(vars.jewelManipulatorStoredPosition);
//    jewelRotator.setPosition(vars.jewelRotatorStoredPosition);
//    threadedRodLift.setPower(0);
    // Relic Grabber

//    if(gamepad2.y && gamepad1.y)
//      slideStop.setPosition(vars.slideStopOpen);
//    else
//      slideStop.setPosition(vars.slideStopClosed);
//    if (gamepad2.a)
//      relicGrabber.setPosition(vars.relicGrabberClosed);
//    else if(gamepad2.b)
//      relicGrabber.setPosition(vars.relicGrabberOpen);
  }

//  private Orientation orientation(){
//    return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//  }
//
//  private float[] imuAngles(){
//    Orientation angles = imu.getAngularOrientation();
//    return new float[]{angles.firstAngle, angles.secondAngle, angles.thirdAngle};
//  }
//
//  private float imuXAngle(){
//    return imuAngles()[0];
//  }
//  private float imuYAngle(){
//    return imuAngles()[1];
//  }
//  private float imuZAngle(){
//    return imuAngles()[2];
//  }
}
