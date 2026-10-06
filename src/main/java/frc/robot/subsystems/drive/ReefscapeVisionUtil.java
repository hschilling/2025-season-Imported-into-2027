package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Inches;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import frc.robot.CommandFactory.RobotPosition;

public class ReefscapeVisionUtil {
        private static final Pose2d TAG_6 = new Pose2d(13.474, 3.306, Rotation2d.fromDegrees(-60));
        private static final Pose2d TAG_7 = new Pose2d(13.89, 4.026, Rotation2d.kZero);
        private static final Pose2d TAG_8 = new Pose2d(13.474, 4.745, Rotation2d.fromDegrees(60));
        private static final Pose2d TAG_9 = new Pose2d(12.643, 4.745, Rotation2d.fromDegrees(-240));
        private static final Pose2d TAG_10 = new Pose2d(12.227, 4.026, Rotation2d.k180deg);
        private static final Pose2d TAG_11 = new Pose2d(12.643, 3.306, Rotation2d.fromDegrees(240));

        private static final Pose2d TAG_17 = new Pose2d(4.074, 3.306, Rotation2d.fromDegrees(-120));
        private static final Pose2d TAG_18 = new Pose2d(3.658, 4.026, Rotation2d.k180deg);
        private static final Pose2d TAG_19 = new Pose2d(4.074, 4.745, Rotation2d.fromDegrees(120));
        private static final Pose2d TAG_20 = new Pose2d(4.905, 4.745, Rotation2d.fromDegrees(60));
        private static final Pose2d TAG_21 = new Pose2d(5.321, 4.026, Rotation2d.kZero);
        private static final Pose2d TAG_22 = new Pose2d(4.905, 3.306, Rotation2d.fromDegrees(-60));

        private static final Transform2d REEF_TO_ROBOT = new Transform2d(Inches.of(29.5), Inches.zero(),
                        Rotation2d.k180deg);
        private static final Transform2d SCORING_POSE_OFFSET_LEFT = new Transform2d(Inches.zero(), Inches.of(6.5),
                        Rotation2d.kZero);
        private static final Transform2d SCORING_POSE_OFFSET_RIGHT = new Transform2d(Inches.zero(), Inches.of(-6.5),
                        Rotation2d.kZero);

        private static final Pose2d RED_REEF_A = TAG_7.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_B = TAG_7.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_C = TAG_8.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_D = TAG_8.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_E = TAG_9.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_F = TAG_9.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_G = TAG_10.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_H = TAG_10.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_I = TAG_11.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_J = TAG_11.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_K = TAG_6.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_L = TAG_6.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));

        private static final Pose2d BLUE_REEF_A = TAG_18.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_B = TAG_18.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_C = TAG_19.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_D = TAG_19.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_E = TAG_20.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_F = TAG_20.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_G = TAG_21.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_H = TAG_21.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_I = TAG_22.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_J = TAG_22.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_K = TAG_17.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_L = TAG_17.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));

        private static final List<Pose2d> LEFT_POSES_RED = Arrays.asList(
                        RED_REEF_A, RED_REEF_C, RED_REEF_E, RED_REEF_G, RED_REEF_I, RED_REEF_K);

        private static final List<Pose2d> RIGHT_POSES_RED = Arrays.asList(
                        RED_REEF_B, RED_REEF_D, RED_REEF_F, RED_REEF_H, RED_REEF_J, RED_REEF_L);

        private static final List<Pose2d> LEFT_POSES_BLUE = Arrays.asList(
                        BLUE_REEF_A, BLUE_REEF_C, BLUE_REEF_E, BLUE_REEF_G, BLUE_REEF_I, BLUE_REEF_K);

        private static final List<Pose2d> RIGHT_POSES_BLUE = Arrays.asList(
                        BLUE_REEF_B, BLUE_REEF_D, BLUE_REEF_F, BLUE_REEF_H, BLUE_REEF_J, BLUE_REEF_L);

        public static Supplier<Pose2d> getGoalPose(RobotPosition robotPosition, Supplier<Pose2d> robotPose,
                        BooleanSupplier isBlueAlliance) {
                Pose2d goalPose;
                if (robotPose.get() == null) {
                        Pose2d nullReturn = new Pose2d();
                        return () -> nullReturn;
                }
                if (isBlueAlliance.getAsBoolean()) {
                        if (robotPosition == RobotPosition.LEFT) {
                                goalPose = robotPose.get().nearest(LEFT_POSES_BLUE);
                        } else {
                                goalPose = robotPose.get().nearest(RIGHT_POSES_BLUE);
                        }
                } else {
                        if (robotPosition == RobotPosition.LEFT) {
                                goalPose = robotPose.get().nearest(LEFT_POSES_RED);
                        } else {
                                goalPose = robotPose.get().nearest(RIGHT_POSES_RED);
                        }
                }
                Supplier<Pose2d> goalPoseSupplier = () -> goalPose;
                return goalPoseSupplier;
        }
}
