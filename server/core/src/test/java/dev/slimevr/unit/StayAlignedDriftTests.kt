package dev.slimevr.unit

import dev.slimevr.math.Angle
import dev.slimevr.tracking.processor.stayaligned.adjust.TrackerYaw
import dev.slimevr.tracking.processor.stayaligned.trackers.YawDriftEstimator
import io.github.axisangles.ktmath.EulerAngles
import io.github.axisangles.ktmath.EulerOrder
import io.github.axisangles.ktmath.Quaternion
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test
import kotlin.math.abs

/**
 * Tests the drift model that lets Stay Aligned cancel drift while moving.
 */
class StayAlignedDriftTests {

	private fun feed(
		estimator: YawDriftEstimator,
		driftRateRadPerSec: Float,
		seconds: Float,
		noiseRad: Float = 0.0f,
		dtSeconds: Float = 0.001f,
	) {
		var tick = 0
		val random = java.util.Random(1234L)
		while (tick * dtSeconds < seconds) {
			val time = tick * dtSeconds
			val noise = if (noiseRad == 0.0f) {
				0.0f
			} else {
				noiseRad * if (random.nextBoolean()) 1.0f else -1.0f
			}
			estimator.observe(Angle.ofRad(driftRateRadPerSec * time + noise), dtSeconds)
			estimator.tick(dtSeconds)
			tick++
		}
	}

	@Test
	fun `estimates positive yaw drift`() {
		val estimator = YawDriftEstimator()
		feed(estimator, 0.001f, 60.0f)

		Assertions.assertTrue(estimator.hasEstimate())
		Assertions.assertEquals(0.001f, estimator.rate, 1e-6f)
		// Correction must oppose the drift and have the right magnitude.
		Assertions.assertEquals(-0.0001f, estimator.correction(0.1f).toRad(), 1e-6f)
	}

	@Test
	fun `estimates negative yaw drift`() {
		val estimator = YawDriftEstimator()
		feed(estimator, -0.002f, 60.0f)

		Assertions.assertEquals(-0.002f, estimator.rate, 1e-5f)
		Assertions.assertEquals(0.0002f, estimator.correction(0.1f).toRad(), 1e-6f)
	}

	@Test
	fun `quantisation noise does not bias the estimate`() {
		val estimator = YawDriftEstimator()
		// 0.005 deg of alternating yaw noise per tick at 1 kHz.
		feed(estimator, 0.0005f, 120.0f, noiseRad = 0.0001f)

		Assertions.assertEquals(0.0005f, estimator.rate, 0.0001f)
	}

	@Test
	fun `rejects implausible windows`() {
		val estimator = YawDriftEstimator()
		// A 1 radian jump is motion or a wrap, not drift.
		feed(estimator, 0.0f, 1.0f)
		estimator.observe(Angle.ofRad(1.0f), 0.001f)
		feed(estimator, 0.0f, 2.0f)

		Assertions.assertFalse(estimator.hasEstimate())
		Assertions.assertEquals(0.0f, estimator.correction(0.1f).toRad())
	}

	@Test
	fun `begin rest drops the previous locked window`() {
		val estimator = YawDriftEstimator()
		feed(estimator, 0.001f, 1.0f)

		// New lock: raw yaw is re-referenced to zero, so the old error must not
		// leak into the next window.
		estimator.beginRest()
		feed(estimator, 0.001f, 60.0f)

		Assertions.assertTrue(estimator.hasEstimate())
		Assertions.assertEquals(0.001f, estimator.rate, 1e-6f)
	}

	@Test
	fun `stale estimate is ignored`() {
		val estimator = YawDriftEstimator()
		feed(estimator, 0.001f, 60.0f)
		estimator.tick(YawDriftEstimator.STALE_SECONDS + 1.0f)

		Assertions.assertEquals(0.0f, estimator.correction(0.1f).toRad())
	}

	@Test
	fun `reset forgets the estimate`() {
		val estimator = YawDriftEstimator()
		feed(estimator, 0.001f, 60.0f)
		estimator.reset()

		Assertions.assertFalse(estimator.hasEstimate())
		Assertions.assertEquals(0.0f, estimator.correction(0.1f).toRad())
	}

	@Test
	fun `feed-forward cancels accumulated drift`() {
		val estimator = YawDriftEstimator()
		val driftRate = 0.001f
		feed(estimator, driftRate, 60.0f)

		// Tracker moves for two minutes; yaw offset is updated 10 times a second.
		val dt = 0.1f
		var correction = 0.0f
		var time = 0.0f
		while (time < 120.0f) {
			correction += estimator.correction(dt).toRad()
			estimator.tick(dt)
			time += dt
		}

		val drift = driftRate * time
		val residual = drift + correction
		Assertions.assertTrue(
			abs(residual) < abs(drift) * 0.1f,
			"residual drift $residual should be much smaller than $drift",
		)
	}

	@Test
	fun `feed-forward keeps the adjusted yaw aligned`() {
		val estimator = YawDriftEstimator()
		val driftRate = 0.001f
		val locked = Quaternion.IDENTITY
		val dt = 0.001f

		// Learn while at rest: the raw rotation drifts away from the locked one.
		var tick = 0
		while (tick * dt < 60.0f) {
			val time = tick * dt
			val raw = Quaternion.rotationAroundYAxis(driftRate * time) * locked
			estimator.observe(TrackerYaw.yawDifference(raw, locked), dt)
			estimator.tick(dt)
			tick++
		}

		// Move for a minute while applying the correction to the raw rotation.
		var correction = Angle.ZERO
		var time = 0.0f
		while (time < 60.0f) {
			correction += estimator.correction(0.1f)
			estimator.tick(0.1f)
			time += 0.1f
		}
		val raw = Quaternion.rotationAroundYAxis(driftRate * time) * locked
		val adjusted = Quaternion.rotationAroundYAxis(correction.toRad()) * raw
		val residual = TrackerYaw.yawDifference(adjusted, locked).toRad()

		Assertions.assertTrue(
			abs(residual) < driftRate * time * 0.1f,
			"residual $residual should be much smaller than ${driftRate * time}",
		)
	}

	@Test
	fun `yaw difference measures world yaw`() {
		val yaw = 0.2f
		val rotation = Quaternion.rotationAroundYAxis(yaw)

		Assertions.assertEquals(yaw, TrackerYaw.yawDifference(rotation, Quaternion.IDENTITY).toRad(), 1e-5f)
	}

	@Test
	fun `yaw difference works on a tracker on its side`() {
		// Euler YZX with Z=90 deg puts the tracker's X axis along world Y, which is
		// where the naive YZX yaw extraction breaks down.
		val onSide = EulerAngles(EulerOrder.YZX, 0.0f, 0.0f, Math.PI.toFloat() / 2.0f).toQuaternion()
		val yaw = 0.15f
		val drifted = Quaternion.rotationAroundYAxis(yaw) * onSide

		val difference = TrackerYaw.yawDifference(drifted, onSide).toRad()
		Assertions.assertEquals(yaw, difference, 1e-4f)
	}

	@Test
	fun `yaw difference follows the sign of the drift`() {
		val onSide = EulerAngles(EulerOrder.YZX, 0.0f, 0.0f, Math.PI.toFloat() / 2.0f).toQuaternion()
		val yaw = -0.1f
		val drifted = Quaternion.rotationAroundYAxis(yaw) * onSide

		val difference = TrackerYaw.yawDifference(drifted, onSide).toRad()
		Assertions.assertTrue(abs(difference - yaw) < 1e-4f)
	}
}
