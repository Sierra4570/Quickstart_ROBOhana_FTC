package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Meccanum_drive {

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    private BNO055IMUNew imu_hub;

    public void init(HardwareMap hwmap) {
        leftFront = hwmap.get(DcMotor.class, "leftFront");
        leftBack = hwmap.get(DcMotor.class, "leftBack");
        rightFront = hwmap.get(DcMotor.class, "rightFront");
        rightBack = hwmap.get(DcMotor.class, "rightBack");

        imu_hub = hwmap.get(BNO055IMUNew.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu_hub.initialize(new IMU.Parameters(orientationOnRobot));

        //Set motor Directions - 10/15/26
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu_hub.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    //declare drive variables to be called in other classes - 10/15/26
    public void drive(double forward, double strafe, double rotate) {
        double frontleftpower = forward + strafe + rotate;
        double backleftpower = forward - strafe + rotate;
        double frontrightpower = forward - strafe - rotate;
        double backrightpower = forward + strafe - rotate;

        double maxPower = 1.0; //Sets max motor power
        double maxSpeed = 1.0; //Adjusts robot speed - set lower for outreach

        //set maxPower to motors - 10/15/26
        maxPower = Math.max(maxPower, Math.abs(frontleftpower));
        maxPower = Math.max(maxPower, Math.abs(backleftpower));
        maxPower = Math.max(maxPower, Math.abs(frontrightpower));
        maxPower = Math.max(maxPower, Math.abs(backrightpower));

        //Constrain with max speed - 10/15/26
        leftFront.setPower(maxSpeed * (frontleftpower / maxPower));
        leftBack.setPower(maxSpeed * (backleftpower / maxPower));
        rightFront.setPower(maxSpeed * (frontrightpower / maxPower));
        rightBack.setPower(maxSpeed * (backrightpower / maxPower));
    }
}
