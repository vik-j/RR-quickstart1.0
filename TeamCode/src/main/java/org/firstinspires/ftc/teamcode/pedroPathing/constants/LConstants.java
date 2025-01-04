package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.0020052828133361685;
        TwoWheelConstants.strafeTicksToInches = 0.0020052828133361685;
        TwoWheelConstants.forwardY = -6.2742459229;
        TwoWheelConstants.strafeX = 1.643933717;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "leftBack";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "rightFront";
        TwoWheelConstants.forwardEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    }
}




