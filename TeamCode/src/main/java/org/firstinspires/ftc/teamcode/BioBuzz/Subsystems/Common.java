package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

public class Common {
    public static Pose AUTO_END_POSE = new Pose(0,0,0);
    public static Robot robot;
    public static MultipleTelemetry dashTelemetry;
    public static TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public static boolean isRed = false;
}
