import org.hamcrest.MatcherAssert.assertThat
import org.hamcrest.Matchers.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertAll

internal class RotationalPointMassPoseTest {

    /*
     * Large values of elapsed time are used for test case simplicity. Actual simulations
     * would use much smaller values to provide much better resolution.
     */

    @Test
    fun simplePointMass() {
        val initialPose = RotationalPointMassPose(1.0, 1.0, AngularPose())

        val poseT1 = initialPose.updatePose({ 1.0 }, 1.0).pose

        assertAll("pose T1",
            { assertThat("position", poseT1.pose.positionDegrees, `is`(equalTo(0.0))) },
            { assertThat("velocity", poseT1.pose.velocityDegPerS, `is`(equalTo(0.0))) },
            { assertThat("accel", poseT1.pose.accelerationDegPerS2, `is`(equalTo(1.0))) },
            { assertThat("jerk", poseT1.pose.jerkDegPerS3, `is`(equalTo(1.0))) }
        )

        val poseT2 = poseT1.updatePose({ 0.0 }, 1.0).pose

        assertAll("pose T2",
            { assertThat("position", poseT2.pose.positionDegrees, `is`(equalTo(0.0))) },
            { assertThat("velocity", poseT2.pose.velocityDegPerS, `is`(equalTo(1.0))) },
            { assertThat("accel", poseT2.pose.accelerationDegPerS2, `is`(equalTo(0.0))) },
            { assertThat("jerk", poseT2.pose.jerkDegPerS3, `is`(equalTo(-1.0))) }
        )

        val poseT3 = poseT2.updatePose({ 0.0 }, 1.0).pose

        assertAll("pose T3",
            { assertThat("position", poseT3.pose.positionDegrees, `is`(equalTo(1.0))) },
            { assertThat("velocity", poseT3.pose.velocityDegPerS, `is`(equalTo(1.0))) },
            { assertThat("accel", poseT3.pose.accelerationDegPerS2, `is`(equalTo(0.0))) },
            { assertThat("jerk", poseT3.pose.jerkDegPerS3, `is`(equalTo(0.0))) }
        )

        val poseT4 = poseT3.updatePose({ -1.0 }, 1.0).pose

        assertAll("pose T4",
            { assertThat("position", poseT4.pose.positionDegrees, `is`(equalTo(2.0))) },
            { assertThat("velocity", poseT4.pose.velocityDegPerS, `is`(equalTo(1.0))) },
            { assertThat("accel", poseT4.pose.accelerationDegPerS2, `is`(equalTo(-1.0))) },
            { assertThat("jerk", poseT4.pose.jerkDegPerS3, `is`(equalTo(-1.0))) }
        )

        val poseT5 = poseT4.updatePose({ 0.0 }, 1.0).pose

        assertAll("pose T5",
            { assertThat("position", poseT5.pose.positionDegrees, `is`(equalTo(3.0))) },
            { assertThat("velocity", poseT5.pose.velocityDegPerS, `is`(equalTo(0.0))) },
            { assertThat("accel", poseT5.pose.accelerationDegPerS2, `is`(equalTo(0.0))) },
            { assertThat("jerk", poseT5.pose.jerkDegPerS3, `is`(equalTo(1.0))) }
        )
    }

    /**
     * Rotational inertia increases quadratically with respect to arm length
     */
    @Test
    fun fartherAwayPointMass() {
        val initialPose = RotationalPointMassPose(1.0, 2.0, AngularPose())

        val poseT1 = initialPose.updatePose({ 1.0 }, 1.0).pose

        assertAll("pose T1",
            { assertThat("position", poseT1.pose.positionDegrees, `is`(equalTo(0.0))) },
            { assertThat("velocity", poseT1.pose.velocityDegPerS, `is`(equalTo(0.0))) },
            { assertThat("accel", poseT1.pose.accelerationDegPerS2, `is`(closeTo(0.25, 0.00001))) },
            { assertThat("jerk", poseT1.pose.jerkDegPerS3, `is`(closeTo(0.25, 0.00001))) }
        )
    }

    /**
     * Rotational inertia increases linearly with respect to mass length
     */
    @Test
    fun biggerPointMass() {
        val initialPose = RotationalPointMassPose(2.0, 1.0, AngularPose())

        val poseT1 = initialPose.updatePose({ 1.0 }, 1.0).pose

        assertAll("pose T1",
            { assertThat("position", poseT1.pose.positionDegrees, `is`(equalTo(0.0))) },
            { assertThat("velocity", poseT1.pose.velocityDegPerS, `is`(equalTo(0.0))) },
            { assertThat("accel", poseT1.pose.accelerationDegPerS2, `is`(closeTo(0.5, 0.00001))) },
            { assertThat("jerk", poseT1.pose.jerkDegPerS3, `is`(closeTo(0.5, 0.00001))) }
        )
    }

    @Test
    fun currentVelocityMadeAvailableToTorqueSource() {
        val initialPose = RotationalPointMassPose(1.0, 1.0, AngularPose())

        val poseT1 = initialPose.updatePose({ currentVelDegPerS ->
            assertThat(currentVelDegPerS, `is`(equalTo(0.0)))

            1.0
        }, 1.0).pose

        assertAll("pose T1",
            { assertThat("position", poseT1.pose.positionDegrees, `is`(equalTo(0.0))) },
            { assertThat("velocity", poseT1.pose.velocityDegPerS, `is`(equalTo(0.0))) },
            { assertThat("accel", poseT1.pose.accelerationDegPerS2, `is`(equalTo(1.0))) },
            { assertThat("jerk", poseT1.pose.jerkDegPerS3, `is`(equalTo(1.0))) }
        )

        val poseT2 = poseT1.updatePose({ currentVelDegPerS ->
            // Assert that the provided velocity is the same as that in the resulting new pose
            assertThat(currentVelDegPerS, `is`(equalTo(1.0)))

            0.0
        }, 1.0).pose

        assertAll("pose T2",
            { assertThat("position", poseT2.pose.positionDegrees, `is`(equalTo(0.0))) },
            { assertThat("velocity", poseT2.pose.velocityDegPerS, `is`(equalTo(1.0))) },
            { assertThat("accel", poseT2.pose.accelerationDegPerS2, `is`(equalTo(0.0))) },
            { assertThat("jerk", poseT2.pose.jerkDegPerS3, `is`(equalTo(-1.0))) }
        )
    }
}