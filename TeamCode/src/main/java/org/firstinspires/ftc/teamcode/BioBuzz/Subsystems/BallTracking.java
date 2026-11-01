package org.firstinspires.ftc.teamcode.BioBuzz.Subsystems;

import com.qualcomm.robotcore.util.Range;

/** The ball-centering command shared by v1tele and autonomous. */
public final class BallTracking {
    public static final double CLOSE_AREA = 8.0;
    private static final double TURN_KP = -0.02;
    private static final double MAX_TURN = 0.4;
    private static final double APPROACH_SPEED = 0.35;
    private static final double APPROACH_DIRECTION = -1.0;
    private static final double ALIGN_RANGE_DEG = 15.0;

    private BallTracking() { }

    public static Command command(double tx, double ta) {
        double turn = Range.clip(TURN_KP * tx, -MAX_TURN, MAX_TURN);
        double alignment = Math.max(0.0, 1.0 - Math.abs(tx) / ALIGN_RANGE_DEG);
        double forward = ta < CLOSE_AREA ? APPROACH_SPEED * APPROACH_DIRECTION * alignment : 0.0;
        return new Command(forward, turn, ta >= CLOSE_AREA);
    }

    public static final class Command {
        public final double forward;
        public final double turn;
        public final boolean close;

        private Command(double forward, double turn, boolean close) {
            this.forward = forward;
            this.turn = turn;
            this.close = close;
        }
    }
}
