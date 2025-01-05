package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.0020052828133361685;
        TwoWheelConstants.strafeTicksToInches = 0.0020052828133361685;
        TwoWheelConstants.forwardY = -1.25;
        TwoWheelConstants.strafeX = 2.5;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "leftBack";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "rightFront";
        TwoWheelConstants.forwardEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    }
}




