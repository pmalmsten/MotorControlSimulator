import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

class AngularPose(val positionDegrees: Double,
                  val velocityDegPerS: Double,
                  val accelerationDegPerS2: Double,
                  val jerkDegPerS3: Double) {
    constructor() : this(0.0,0.0,0.0,0.0)
}

class DCPermanentMagnetMotor(val maxTorqueNM: Double,
                             val maxVelDegPerS: Double) {

    init {
        if (maxVelDegPerS <= 0.0) error("maxVelDegPerS must be > 0.")
    }

    fun computeTorque(currentDegPerS: Double,
                      currentDirectionalDutyCyclePercent: Double): Double {

        require(currentDirectionalDutyCyclePercent >= -1 && currentDirectionalDutyCyclePercent <= 1)
        { "currentDirectionalDutyCyclePercent must be between -1 and 1" }

        val isReversingDirection = (sign(currentDegPerS) != 0.0
                && sign(currentDirectionalDutyCyclePercent) != 0.0
                && sign(currentDegPerS) != sign(currentDirectionalDutyCyclePercent))

        val reversingSign = if (isReversingDirection) -1 else 1

        val absPercentMaxVel = abs(currentDegPerS) / maxVelDegPerS

        // When a permanent mag. DC motor is 'plugging' (force applied is opposite to current direction of travel),
        // max torque increases as velocity increases (rather than decreases) due to back EMF boost
        val percentMaxTorque = 1 - reversingSign * absPercentMaxVel

        return percentMaxTorque * currentDirectionalDutyCyclePercent * maxTorqueNM
    }
}

class RotationalPointMassPose(
    val massKg: Double,
    val radiusM: Double,
    val pose: AngularPose) {

    private val pointMassAngularInertia = massKg*(radiusM.pow(2))

    fun updatePose(torqueNMToApplyAtNewVelocityDegPerS: (currentVelocityDegPerS: Double) -> Double,
                   elapsedTimeS: Double): RotationalPointMassPose {

        val newVelocityDegPerS = pose.velocityDegPerS + deltaVelocityDegPerS(elapsedTimeS)

        val newAngularAccelDegPerS2 =
            torqueNMToApplyAtNewVelocityDegPerS(newVelocityDegPerS) / pointMassAngularInertia

        val newPose = AngularPose(
            positionDegrees = pose.positionDegrees + deltaPositionDeg(elapsedTimeS),
            velocityDegPerS = newVelocityDegPerS,
            accelerationDegPerS2 = newAngularAccelDegPerS2,
            jerkDegPerS3 = (newAngularAccelDegPerS2 - pose.accelerationDegPerS2) / elapsedTimeS
        )

        return RotationalPointMassPose(massKg, radiusM, newPose)
    }

    private fun deltaPositionDeg(elapsedTimeS: Double): Double {
        return pose.velocityDegPerS * elapsedTimeS
    }

    private fun deltaVelocityDegPerS(elapsedTimeS: Double): Double {
        return pose.accelerationDegPerS2 * elapsedTimeS
    }
}

class MotorAndRotationalPointMassPose(val motor: DCPermanentMagnetMotor,
                                      val massPose: RotationalPointMassPose) {

    val pose = massPose.pose

    fun updatePose(motorDirectionalDutyCyclePercent: Double,
                   elapsedTimeS: Double): MotorAndRotationalPointMassPose {

        val torqueNMToApplyAtNewVelocityDegPerS = { newVelDegPerS: Double ->
            val motorTorqueNM = motor.computeTorque(newVelDegPerS, motorDirectionalDutyCyclePercent)
            val netTorqueNM = motorTorqueNM - simulatedFrictionNMAtVelocity(newVelDegPerS)

            netTorqueNM
        }

        val updatedPose = massPose.updatePose(torqueNMToApplyAtNewVelocityDegPerS, elapsedTimeS)

        return MotorAndRotationalPointMassPose(motor, updatedPose)
    }

    private fun simulatedFrictionNMAtVelocity(currentVelDegPerS: Double): Double {
        // Friction of 10% of max motor torque at max motor speed
        return (currentVelDegPerS/motor.maxVelDegPerS) * (0.10 * motor.maxTorqueNM)
    }
}