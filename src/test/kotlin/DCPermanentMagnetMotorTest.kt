import org.hamcrest.MatcherAssert.assertThat
import org.hamcrest.Matchers.*
import org.junit.jupiter.api.Test

internal class DCPermanentMagnetMotorTest {

    private val maxTorqueNM = 1.0
    private val maxVelRadPerS = 30.0
    private val maxAllowedError = 0.00001

    private val motor = DCPermanentMagnetMotor(maxTorqueNM, maxVelRadPerS)

    @Test
    fun motorTorqueAtStopIsMaxTorque_forwardDirection() {
        assertThat(
            motor.computeTorque(0.0, 1.0),
            `is`(closeTo(maxTorqueNM, maxAllowedError)))
    }

    @Test
    fun motorTorqueAtStopIsMaxTorque_reverseDirection() {
        assertThat(
            motor.computeTorque(0.0, -1.0),
            `is`(closeTo(-maxTorqueNM, maxAllowedError)))
    }

    @Test
    fun motorTorqueAt50PercentMaxVelIs50PercentMaxTorque_forwardDirection() {
        assertThat(
            motor.computeTorque(maxVelRadPerS / 2.0, 1.0),
            `is`(closeTo(maxTorqueNM / 2.0, maxAllowedError)))
    }

    @Test
    fun motorTorqueAt50PercentMaxVelIs50PercentMaxTorque_reverseDirection() {
        assertThat(
            motor.computeTorque(-maxVelRadPerS / 2.0, -1.0),
            `is`(closeTo(-maxTorqueNM / 2.0, maxAllowedError)))
    }

    @Test
    fun motorTorqueAt50PercentMaxVelAnd20PercentDutyIs10PercentMaxTorque_forwardDirection() {
        // At 50% max speed, max torque available is 50%
        // At 20% duty cycle, result torque is 20% * 50% = 10% max torque
        assertThat(
            motor.computeTorque(maxVelRadPerS / 2.0, 0.2),
            `is`(closeTo(maxTorqueNM / 10.0, maxAllowedError)))
    }

    @Test
    fun motorTorqueAt50PercentMaxVelAnd20PercentDutyIs10PercentMaxTorque_reverseDirection() {
        // At 50% max speed, max torque available is 50%
        // At 20% duty cycle, result torque is 20% * 50% = 10% max torque
        assertThat(
            motor.computeTorque(-maxVelRadPerS / 2.0, -0.2),
            `is`(closeTo(-maxTorqueNM / 10.0, maxAllowedError)))
    }

    @Test
    fun motorTorqueAt50PercentMaxVelInReverseIs150PercentMaxTorqueForward() {
        // When a permanent mag. DC motor is 'plugging' (force applied is opposite to current direction of travel),
        // max torque increases as velocity increases (rather than decreases) due to back EMF boost
        assertThat(
            motor.computeTorque(-maxVelRadPerS / 2.0, 1.0),
            `is`(closeTo(1.5*maxTorqueNM, maxAllowedError)))
    }

    @Test
    fun motorTorqueAt50PercentMaxVelForwardIs150PercentMaxTorqueReverse() {
        // When a permanent mag. DC motor is 'plugging' (force applied is opposite to current direction of travel),
        // max torque increases as velocity increases (rather than decreases) due to back EMF boost
        assertThat(
            motor.computeTorque(maxVelRadPerS / 2.0, -1.0),
            `is`(closeTo(-1.5*maxTorqueNM, maxAllowedError)))
    }
}