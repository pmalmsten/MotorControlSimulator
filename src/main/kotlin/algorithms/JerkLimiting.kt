package algorithms

import java.util.concurrent.TimeUnit


class JerkLimiter(
    private val maxDeltaTorquePercentPerSecond: Double,
    initialTimestampNanos: Long,
    initialForcePercent: Double) {

    constructor(maxDeltaTorquePercentPerSecond: Double) :
            this(maxDeltaTorquePercentPerSecond, 0, 0.0)

    var lastResultForcePercent = initialForcePercent
    var lastUpdateTimeNanos = initialTimestampNanos

    /**
     * Constrains change in the returned value to be at most this object's max
     * change per second.
     */
    fun limitJerk(desiredTorquePercent: Double,
                  currentTimeNanos: Long): Double {

        val timeDeltaSeconds = (currentTimeNanos - lastUpdateTimeNanos) / TimeUnit.SECONDS.toNanos(1).toDouble()
        val allowedTorquePercentDelta = maxDeltaTorquePercentPerSecond * timeDeltaSeconds

        val result =  desiredTorquePercent.coerceIn(
            minimumValue = lastResultForcePercent - allowedTorquePercentDelta,
            maximumValue = lastResultForcePercent + allowedTorquePercentDelta)

        lastResultForcePercent = result
        lastUpdateTimeNanos = currentTimeNanos

        return result
    }
}