import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

class AngularPose(val positionRad: Double,
                  val velocityRadPerS: Double,
                  val accelerationRadPerS2: Double,
                  val jerkRadPerS3: Double) {
    constructor() : this(0.0,0.0,0.0,0.0)
}

class DCPermanentMagnetMotor(val maxTorqueNM: Double,
                             val maxVelRadPerS: Double) {

    init {
        if (maxVelRadPerS <= 0.0) error("maxVelRadPerS must be > 0.")
    }

    fun computeTorque(currentRadPerS: Double,
                      currentDirectionalDutyCyclePercent: Double): Double {

        require(currentDirectionalDutyCyclePercent >= -1 && currentDirectionalDutyCyclePercent <= 1)
        { "currentDirectionalDutyCyclePercent must be between -1 and 1" }

        val isReversingDirection = (sign(currentRadPerS) != 0.0
                && sign(currentDirectionalDutyCyclePercent) != 0.0
                && sign(currentRadPerS) != sign(currentDirectionalDutyCyclePercent))

        val reversingSign = if (isReversingDirection) -1 else 1

        val absPercentMaxVel = abs(currentRadPerS) / maxVelRadPerS

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

    fun updatePose(torqueNMToApplyAtNewVelocityRadPerS: (currentVelocityRadPerS: Double) -> Double,
                   elapsedTimeS: Double): UpdatePoseResult {

        val newVelocityRadPerS = pose.velocityRadPerS + deltaVelocityRadPerS(elapsedTimeS)

        val computedTorqueToApplyNM = torqueNMToApplyAtNewVelocityRadPerS(newVelocityRadPerS)
        val newAngularAccelRadPerS2 = computedTorqueToApplyNM / pointMassAngularInertia

        val newPose = AngularPose(
            positionRad = pose.positionRad + deltaPositionRad(elapsedTimeS),
            velocityRadPerS = newVelocityRadPerS,
            accelerationRadPerS2 = newAngularAccelRadPerS2,
            jerkRadPerS3 = (newAngularAccelRadPerS2 - pose.accelerationRadPerS2) / elapsedTimeS
        )

        return UpdatePoseResult(RotationalPointMassPose(massKg, radiusM, newPose),
            computedTorqueToApplyNM)
    }

    private fun deltaPositionRad(elapsedTimeS: Double): Double {
        return pose.velocityRadPerS * elapsedTimeS
    }

    private fun deltaVelocityRadPerS(elapsedTimeS: Double): Double {
        return pose.accelerationRadPerS2 * elapsedTimeS
    }

    data class UpdatePoseResult(val pose: RotationalPointMassPose,
                                val computedTorqueNM: Double)
}

class MotorAndRotationalPointMassPose(val motor: DCPermanentMagnetMotor,
                                      val massPose: RotationalPointMassPose) {

    val pose = massPose.pose

    fun updatePose(motorDirectionalDutyCyclePercent: Double,
                   elapsedTimeS: Double): UpdatePoseResult {

        val torqueNMToApplyAtNewVelocityRadPerS = { newVelRadPerS: Double ->
            val motorTorqueNM = motor.computeTorque(newVelRadPerS, motorDirectionalDutyCyclePercent)
            val netTorqueNM = motorTorqueNM - simulatedFrictionNMAtVelocity(newVelRadPerS)

            netTorqueNM
        }

        val updatePoseResult = massPose.updatePose(torqueNMToApplyAtNewVelocityRadPerS, elapsedTimeS)

        return UpdatePoseResult(MotorAndRotationalPointMassPose(motor, updatePoseResult.pose),
            updatePoseResult.computedTorqueNM)

    }

    private fun simulatedFrictionNMAtVelocity(currentVelRadPerS: Double): Double {
        // Friction of 10% of max motor torque at max motor speed
        return (currentVelRadPerS/motor.maxVelRadPerS) * (0.10 * motor.maxTorqueNM)
    }

    data class UpdatePoseResult(val pose: MotorAndRotationalPointMassPose,
                                val computedTorqueNM: Double)
}