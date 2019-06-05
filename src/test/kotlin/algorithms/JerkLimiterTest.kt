package algorithms

import org.hamcrest.MatcherAssert.assertThat
import org.hamcrest.Matchers.`is`
import org.hamcrest.Matchers.equalTo
import org.junit.jupiter.api.Test
import java.util.concurrent.TimeUnit

internal class JerkLimiterTest {

    private val limiter = JerkLimiter(
        maxDeltaTorquePercentPerSecond = 0.5,
        initialForcePercent = 0.0,
        initialTimestampNanos = 0
    )

    @Test
    fun firstValue_positive_valueWithinLimitPassedThrough() {
        val limitedForcePercent = limiter.limitJerk(0.23, TimeUnit.SECONDS.toNanos(1))

        assertThat(limitedForcePercent, `is`(equalTo(0.23)))
    }

    @Test
    fun firstValue_positive() {
        val limitedForcePercent = limiter.limitJerk(1.0, TimeUnit.SECONDS.toNanos(1))

        assertThat(limitedForcePercent, `is`(equalTo(0.5)))
    }

    @Test
    fun firstValue_negative() {
        val limitedForcePercent = limiter.limitJerk(-1.0, TimeUnit.SECONDS.toNanos(1))

        assertThat(limitedForcePercent, `is`(equalTo(-0.5)))
    }

    @Test
    fun secondValue_positive() {
        limiter.limitJerk(10.0, TimeUnit.SECONDS.toNanos(1))
        val limitedForcePercent = limiter.limitJerk(2.0, TimeUnit.SECONDS.toNanos(2))

        // First value allowed 0.5, second allows 0.5 more than last time
        assertThat(limitedForcePercent, `is`(equalTo(1.0)))
    }

    @Test
    fun secondValue_negative() {
        limiter.limitJerk(-10.0, TimeUnit.SECONDS.toNanos(1))
        val limitedForcePercent = limiter.limitJerk(-2.0, TimeUnit.SECONDS.toNanos(2))

        // First value allowed -0.5, second allows -0.5 more than last time
        assertThat(limitedForcePercent, `is`(equalTo(-1.0)))
    }
}