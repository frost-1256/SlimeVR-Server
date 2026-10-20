package dev.slimevr.unit

import dev.slimevr.filtering.predictRotation
import io.github.axisangles.ktmath.Quaternion
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import kotlin.math.PI

class QuaternionPredictionTests {
	private fun yaw(degrees: Float) = Quaternion.rotationAroundYAxis(degrees * PI.toFloat() / 180f)

	@Test
	fun `replays the newest delta first`() {
		val slow = yaw(1f)
		val fast = yaw(3f)
		val latest = yaw(10f)
		// Oldest to newest: the tracker sped up from 1 to 3 degrees per sample.
		val deltas = listOf(slow, slow, slow, fast, fast, fast)

		val expected = latest * fast * fast * fast * slow * slow * slow
		val actual = predictRotation(deltas, latest)

		assertTrue(
			TrackerTestUtils.quatApproxEqual(expected, actual, 1e-4f),
			"expected <$expected>, actual <$actual>",
		)
	}

	@Test
	fun `no deltas keeps the latest rotation`() {
		val latest = yaw(42f)
		assertTrue(TrackerTestUtils.quatApproxEqual(latest, predictRotation(emptyList(), latest)))
	}
}
