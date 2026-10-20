package dev.slimevr.unit

import com.fasterxml.jackson.databind.ObjectMapper
import dev.slimevr.config.RecordingConfig
import dev.slimevr.math.Angle
import dev.slimevr.recording.LiveDataRecorder
import io.github.axisangles.ktmath.EulerAngles
import io.github.axisangles.ktmath.EulerOrder
import io.github.axisangles.ktmath.Quaternion
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.io.TempDir
import java.io.File
import java.net.URI
import java.net.http.HttpClient
import java.net.http.HttpRequest
import java.net.http.HttpResponse
import java.nio.file.Path

class LiveDataRecorderTests {

	private val mapper = ObjectMapper()

	@Test
	fun `samples contain raw and corrected rotations`(@TempDir dir: Path) {
		val trackers = TestTrackerSet()
		val hip = trackers.hip
		val rotation = EulerAngles(EulerOrder.YZX, 0.1f, 0.2f, 0.3f).toQuaternion()
		hip.setRotation(rotation)
		hip.stayAligned.yawCorrection = Angle.ofDeg(2.5f)

		val recorder = recorder(dir, trackers.allL)
		try {
			assertNotNull(recorder.startRecording())
			recorder.sampleOnce()
			assertTrue(recorder.stopRecording())

			val lines = recordingFiles(dir).single().readLines()
			val header = mapper.readTree(lines[0])
			assertEquals("session", header["type"].asText())
			assertEquals(1, header["version"].asInt())

			val sample = lines.map { mapper.readTree(it) }.first {
				it["type"].asText() == "sample" && it["tracker"].asText() == "HIP"
			}
			assertEquals("HIP", sample["tracker"].asText())
			assertEquals(hip.id, sample["id"].asInt())
			assertEquals("OK", sample["status"].asText())
			assertEquals(2.5, sample["yawCorrection"].asDouble(), 1e-4)
			assertFalse(sample["rest"].asBoolean())
			assertEquals(4, sample["raw"].size())
			assertEquals(rotation.x.toDouble(), sample["raw"][0].asDouble(), 1e-5)
			assertEquals(rotation.w.toDouble(), sample["raw"][3].asDouble(), 1e-5)
			// The adjusted rotation includes the yaw correction.
			assertTrue(sample["adjusted"][1].asDouble() != sample["raw"][1].asDouble())

			assertEquals("end", mapper.readTree(lines.last())["type"].asText())
		} finally {
			recorder.close()
		}
	}

	@Test
	fun `reset events are written while recording`(@TempDir dir: Path) {
		val trackers = TestTrackerSet()
		val recorder = recorder(dir, listOf(trackers.hip))
		try {
			recorder.startRecording()
			trackers.hip.resetsHandler.resetYaw(Quaternion.IDENTITY)
			recorder.sampleOnce()
			recorder.stopRecording()

			val events = recordingFiles(dir).single().readLines()
				.map { mapper.readTree(it) }
				.filter { it["type"].asText() == "event" }
			assertEquals(1, events.size)
			assertEquals("reset_yaw", events.single()["event"].asText())
			assertEquals("HIP", events.single()["tracker"].asText())
		} finally {
			recorder.close()
		}
	}

	@Test
	fun `http page starts and downloads recordings`(@TempDir dir: Path) {
		val trackers = TestTrackerSet()
		val recorder = recorder(dir, listOf(trackers.hip))
		try {
			recorder.startService()
			val port = recorder.servicePort()
			assertNotNull(port)
			val base = "http://127.0.0.1:$port"

			assertFalse(status(base)["recording"].asBoolean())
			assertTrue(start(base)["recording"].asBoolean())

			val files = mapper.readTree(get(base, "/files"))
			assertEquals(1, files.size())
			val name = files[0]["name"].asText()

			val download = getRaw(base, "/files/$name")
			assertEquals(200, download.first)
			val header = mapper.readTree(download.second.toString(Charsets.UTF_8).lineSequence().first())
			assertEquals("session", header["type"].asText())

			assertFalse(stop(base)["recording"].asBoolean())
			assertEquals(404, getRaw(base, "/files/nope.jsonl").first)
			assertEquals(404, getRaw(base, "/files/../config.json").first)
		} finally {
			recorder.close()
		}
	}

	private fun recorder(dir: Path, trackers: List<dev.slimevr.tracking.trackers.Tracker>) = LiveDataRecorder(
		RecordingConfig().apply {
			directory = dir.toFile().absolutePath
			port = 0
			sampleRateHz = 1
		},
		{ trackers },
	)

	private fun recordingFiles(dir: Path): List<File> = dir.toFile()
		.listFiles { file -> file.name.endsWith(".jsonl") }
		?.sorted()
		.orEmpty()

	private fun status(base: String) = mapper.readTree(get(base, "/status"))

	private fun start(base: String) = mapper.readTree(post(base, "/start"))

	private fun stop(base: String) = mapper.readTree(post(base, "/stop"))

	private fun get(base: String, path: String) = getRaw(base, path).second.toString(Charsets.UTF_8)

	private fun post(base: String, path: String): String {
		val request = HttpRequest
			.newBuilder(URI.create(base + path))
			.POST(HttpRequest.BodyPublishers.noBody())
			.build()
		return HttpClient.newHttpClient()
			.send(request, HttpResponse.BodyHandlers.ofString())
			.body()
	}

	private fun getRaw(base: String, path: String): Pair<Int, ByteArray> {
		val request = HttpRequest.newBuilder(URI.create(base + path)).GET().build()
		val response = HttpClient.newHttpClient()
			.send(request, HttpResponse.BodyHandlers.ofByteArray())
		return response.statusCode() to response.body()
	}
}
