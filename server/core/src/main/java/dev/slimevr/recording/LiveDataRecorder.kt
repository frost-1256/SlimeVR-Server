package dev.slimevr.recording

import com.fasterxml.jackson.core.JsonFactory
import com.fasterxml.jackson.core.JsonGenerator
import com.fasterxml.jackson.core.io.SerializedString
import com.sun.net.httpserver.HttpExchange
import com.sun.net.httpserver.HttpServer
import dev.slimevr.SLIMEVR_IDENTIFIER
import dev.slimevr.config.DriftCompensationConfig
import dev.slimevr.config.RecordingConfig
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.udp.MagnetometerStatus
import io.eiren.util.OperatingSystem
import io.eiren.util.logging.LogManager
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter
import java.io.IOException
import java.io.StringWriter
import java.net.InetAddress
import java.net.InetSocketAddress
import java.net.URLDecoder
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit
import kotlin.math.PI

/**
 * Records live tracker state to JSON Lines files so drift and accuracy can be
 * analysed from real hardware instead of synthetic tests.
 *
 * A page on localhost starts and stops recordings and serves the files:
 * `http://127.0.0.1:21111/`.
 *
 * Every sample contains the raw, filtered and identity-adjusted rotations, the
 * Stay Aligned yaw correction and drift rate, the drift compensation state, and
 * reset events. Recording happens on a dedicated thread and only while a
 * session is active.
 */
class LiveDataRecorder(
	private val config: RecordingConfig,
	private val trackers: () -> List<Tracker>,
	private val driftConfig: () -> DriftCompensationConfig? = { null },
) {
	private val json = JsonFactory()
	private val lock = Any()
	private val scheduler: ScheduledExecutorService = Executors
		.newSingleThreadScheduledExecutor { runnable ->
			Thread(runnable, "LiveDataRecorder").apply { isDaemon = true }
		}

	private var http: HttpServer? = null
	private var generator: JsonGenerator? = null
	private var writer: BufferedWriter? = null
	private var currentFile: File? = null
	private var task: ScheduledFuture<*>? = null
	private var startNanos = 0L
	private var samples = 0L

	private val periodMs: Long = (1000 / config.sampleRateHz.coerceIn(1, 240)).toLong()

	/** Folder that holds the recordings. */
	val directory: File by lazy {
		val configured = config.directory?.takeIf { it.isNotBlank() }?.let { File(it) }
		configured
			?: OperatingSystem.resolveLogDirectory(SLIMEVR_IDENTIFIER)?.resolve("recordings")?.toFile()
			?: File("recordings").absoluteFile
	}

	init {
		active = this
	}

	/** Starts the local HTTP page, unless disabled in the config. */
	fun startService() {
		if (!config.enabled) {
			LogManager.info("[LiveDataRecorder] Disabled in the config")
			return
		}
		if (http != null) return

		try {
			val server = HttpServer.create(
				InetSocketAddress(InetAddress.getByName("127.0.0.1"), config.port.coerceIn(0, 65535)),
				0,
			)
			server.createContext("/", ::handleHttp)
			server.executor = Executors.newFixedThreadPool(2) { runnable ->
				Thread(runnable, "LiveDataRecorder HTTP").apply { isDaemon = true }
			}
			server.start()
			http = server
			Runtime.getRuntime().addShutdownHook(Thread(::close, "LiveDataRecorder shutdown"))
			LogManager.info(
				"[LiveDataRecorder] Open http://127.0.0.1:${server.address.port}/ to record live data " +
					"(folder: ${directory.absolutePath})",
			)
		} catch (e: Exception) {
			LogManager.warning(
				"[LiveDataRecorder] HTTP service failed on port ${config.port}: ${e.message}",
			)
		}
	}

	/** Port the page is listening on, or null when the service is off. */
	fun servicePort(): Int? = http?.address?.port

	/** Starts a new recording file. Returns its name, or null when already recording. */
	fun startRecording(): String? = synchronized(lock) {
		if (writer != null) return null

		directory.mkdirs()
		val file = nextFile()
		val out = try {
			BufferedWriter(FileWriter(file, true), 64 * 1024)
		} catch (e: IOException) {
			LogManager.warning("[LiveDataRecorder] Cannot create ${file.absolutePath}: ${e.message}")
			return null
		}
		val gen = json.createGenerator(out)
		gen.setRootValueSeparator(SerializedString("\n"))
		try {
			writeHeader(gen)
			gen.flush()
		} catch (e: IOException) {
			runCatching { gen.close() }
			LogManager.warning("[LiveDataRecorder] Cannot write ${file.absolutePath}: ${e.message}")
			return null
		}

		generator = gen
		writer = out
		currentFile = file
		startNanos = System.nanoTime()
		samples = 0
		task = scheduler.scheduleAtFixedRate(
			{ sampleSafely() },
			periodMs,
			periodMs,
			TimeUnit.MILLISECONDS,
		)
		LogManager.info("[LiveDataRecorder] Recording to ${file.absolutePath}")
		file.name
	}

	/** Stops the active recording and closes its file. */
	fun stopRecording(): Boolean = synchronized(lock) {
		if (writer == null) return false
		writeTrailer()
		closeSession()
		true
	}

	fun isRecording(): Boolean = synchronized(lock) { writer != null }

	/** Appends one sample for every current tracker. Also used by tests. */
	fun sampleOnce() {
		val snapshot = trackers()
		synchronized(lock) {
			val gen = generator ?: return
			val elapsed = (System.nanoTime() - startNanos) / 1_000_000_000.0
			val wall = System.currentTimeMillis()
			try {
				for (tracker in snapshot) {
					writeSample(gen, tracker, elapsed, wall)
				}
				gen.flush()
				samples++
			} catch (e: IOException) {
				LogManager.warning("[LiveDataRecorder] Recording stopped after an I/O error: ${e.message}")
				closeSession()
				return
			}
			val file = currentFile
			if (file != null && file.length() >= config.maxFileBytes.coerceAtLeast(1024)) {
				LogManager.info("[LiveDataRecorder] Size limit reached, stopping ${file.name}")
				writeTrailer()
				closeSession()
			}
		}
	}

	/** Writes a reset or calibration event to the active recording. */
	fun recordEvent(tracker: Tracker, event: String) {
		synchronized(lock) {
			val gen = generator ?: return
			val elapsed = (System.nanoTime() - startNanos) / 1_000_000_000.0
			try {
				gen.writeStartObject()
				gen.writeStringField("type", "event")
				gen.writeStringField("event", event)
				gen.writeNumberField("t", elapsed)
				gen.writeNumberField("wall", System.currentTimeMillis())
				gen.writeStringField("tracker", tracker.name)
				gen.writeNumberField("id", tracker.id)
				gen.writeEndObject()
				gen.flush()
			} catch (e: IOException) {
				LogManager.warning("[LiveDataRecorder] Cannot write event: ${e.message}")
				closeSession()
			}
		}
	}

	/** Stops recording and the HTTP page. */
	fun close() {
		synchronized(lock) {
			writeTrailer()
			closeSession()
		}
		http?.stop(0)
		http = null
		scheduler.shutdownNow()
	}

	private fun sampleSafely() {
		try {
			sampleOnce()
		} catch (e: Exception) {
			LogManager.warning("[LiveDataRecorder] Sampling failed: ${e.message}")
			synchronized(lock) {
				closeSession()
			}
		}
	}

	private fun closeSession() {
		task?.cancel(false)
		task = null
		runCatching { generator?.flush() }
		runCatching { writer?.newLine() }
		runCatching { generator?.close() }
		runCatching { writer?.close() }
		generator = null
		writer = null
		val file = currentFile
		currentFile = null
		if (file != null) {
			LogManager.info("[LiveDataRecorder] Stopped ${file.name} (${file.length()} bytes)")
		}
	}

	private fun nextFile(): File {
		val stamp = DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss").format(LocalDateTime.now())
		var index = 0
		while (true) {
			val suffix = if (index == 0) "" else "-$index"
			val file = File(directory, "live-$stamp$suffix.jsonl")
			if (!file.exists()) return file
			index++
		}
	}

	private fun writeHeader(gen: JsonGenerator) {
		gen.writeStartObject()
		gen.writeStringField("type", "session")
		gen.writeNumberField("version", 1)
		gen.writeNumberField("startedAt", System.currentTimeMillis())
		gen.writeNumberField("intervalMs", periodMs)
		gen.writeArrayFieldStart("trackers")
		for (tracker in trackers()) {
			gen.writeStartObject()
			gen.writeStringField("name", tracker.name)
			gen.writeNumberField("id", tracker.id)
			gen.writeNumberField("device", tracker.device?.id ?: 0)
			gen.writeStringField("role", tracker.trackerPosition?.name ?: "")
			gen.writeEndObject()
		}
		gen.writeEndArray()
		driftConfig()?.let { drift ->
			gen.writeObjectFieldStart("driftCompensation")
			gen.writeBooleanField("enabled", drift.enabled)
			gen.writeBooleanField("prediction", drift.prediction)
			gen.writeNumberField("amount", drift.amount)
			gen.writeNumberField("maxResets", drift.maxResets)
			gen.writeEndObject()
		}
		gen.writeEndObject()
	}

	private fun writeTrailer() {
		val gen = generator ?: return
		val elapsed = (System.nanoTime() - startNanos) / 1_000_000_000.0
		runCatching {
			gen.writeStartObject()
			gen.writeStringField("type", "end")
			gen.writeNumberField("t", elapsed)
			gen.writeNumberField("samples", samples)
			gen.writeEndObject()
			gen.flush()
		}
	}

	private fun writeSample(gen: JsonGenerator, tracker: Tracker, elapsed: Double, wall: Long) {
		gen.writeStartObject()
		gen.writeStringField("type", "sample")
		gen.writeNumberField("t", elapsed)
		gen.writeNumberField("wall", wall)
		gen.writeStringField("tracker", tracker.name)
		gen.writeNumberField("id", tracker.id)
		tracker.device?.let { gen.writeNumberField("device", it.id) }
		tracker.trackerPosition?.let { gen.writeStringField("role", it.name) }
		gen.writeStringField("status", tracker.status.name)
		gen.writeStringField("data", tracker.trackerDataType.name)
		tracker.imuType?.let { gen.writeStringField("imu", it.name) }

		gen.writeArrayFieldStart("raw")
		writeQuaternion(gen, tracker.getRawRotation())
		gen.writeEndArray()
		gen.writeArrayFieldStart("adjusted")
		writeQuaternion(gen, tracker.getRotationNoResetSmooth())
		gen.writeEndArray()
		gen.writeArrayFieldStart("identity")
		writeQuaternion(gen, tracker.getIdentityAdjustedRotation())
		gen.writeEndArray()

		gen.writeNumberField("yawCorrection", tracker.stayAligned.yawCorrection.toDeg().toDouble())
		gen.writeNumberField("driftRate", tracker.stayAligned.yawDrift.rate.toDegrees())
		gen.writeArrayFieldStart("driftQuat")
		writeQuaternion(gen, tracker.resetsHandler.driftQuaternion)
		gen.writeEndArray()
		gen.writeNumberField("driftAmount", tracker.resetsHandler.appliedDriftAmount.toDouble())
		gen.writeArrayFieldStart("sinceReset")
		writeQuaternion(gen, tracker.resetsHandler.rotationSinceLastReset)
		gen.writeEndArray()
		gen.writeBooleanField("rest", tracker.stayAligned.lockedRotation != null)
		gen.writeBooleanField("needReset", tracker.needReset)
		gen.writeBooleanField("compensating", tracker.resetsHandler.driftCompensationActive)

		if (tracker.hasPosition) {
			gen.writeArrayFieldStart("position")
			writeVector(gen, tracker.position)
			gen.writeEndArray()
		}
		if (tracker.hasAcceleration) {
			gen.writeArrayFieldStart("accel")
			writeVector(gen, tracker.getAcceleration())
			gen.writeEndArray()
		}
		if (tracker.hasVelocity) {
			gen.writeArrayFieldStart("velocity")
			writeVector(gen, tracker.getVelocity())
			gen.writeEndArray()
		}
		if (tracker.magStatus == MagnetometerStatus.ENABLED) {
			gen.writeArrayFieldStart("mag")
			writeVector(gen, tracker.getMagVector())
			gen.writeEndArray()
		}

		gen.writeEndObject()
	}

	private fun writeQuaternion(gen: JsonGenerator, rotation: Quaternion) {
		gen.writeNumber(rotation.x.toDouble())
		gen.writeNumber(rotation.y.toDouble())
		gen.writeNumber(rotation.z.toDouble())
		gen.writeNumber(rotation.w.toDouble())
	}

	private fun writeVector(gen: JsonGenerator, vector: Vector3) {
		gen.writeNumber(vector.x.toDouble())
		gen.writeNumber(vector.y.toDouble())
		gen.writeNumber(vector.z.toDouble())
	}

	private fun Float.toDegrees() = this * 180.0 / PI

	private fun handleHttp(exchange: HttpExchange) {
		try {
			val path = exchange.requestURI.path
			when {
				path == "/" -> sendHtml(exchange)

				path == "/status" -> sendJson(exchange, statusJson())

				path == "/start" && exchange.requestMethod == "POST" -> {
					startRecording()
					sendJson(exchange, statusJson())
				}

				path == "/stop" && exchange.requestMethod == "POST" -> {
					stopRecording()
					sendJson(exchange, statusJson())
				}

				path == "/files" -> sendJson(exchange, filesJson())

				path.startsWith("/files/") -> sendFile(exchange, path.removePrefix("/files/"))

				else -> send(exchange, 404, "text/plain; charset=utf-8", "not found".toByteArray())
			}
		} catch (e: Exception) {
			runCatching {
				send(exchange, 500, "text/plain; charset=utf-8", "error: ${e.message}".toByteArray())
			}
		} finally {
			exchange.close()
		}
	}

	private fun statusJson(): String {
		val recording = isRecording()
		val file = synchronized(lock) { currentFile }
		val elapsed = if (recording) (System.nanoTime() - startNanos) / 1_000_000_000.0 else 0.0
		return jsonBody { gen ->
			gen.writeBooleanField("recording", recording)
			gen.writeStringField("file", file?.name ?: "")
			gen.writeNumberField("bytes", file?.length() ?: 0)
			gen.writeNumberField("samples", samples)
			gen.writeNumberField("seconds", elapsed)
			gen.writeNumberField("trackers", trackers().size)
			gen.writeStringField("folder", directory.absolutePath)
		}
	}

	private fun filesJson(): String = jsonArrayBody { gen ->
		for (file in listRecordings()) {
			gen.writeStartObject()
			gen.writeStringField("name", file.name)
			gen.writeNumberField("bytes", file.length())
			gen.writeNumberField("modified", file.lastModified())
			gen.writeEndObject()
		}
	}

	private fun listRecordings(): List<File> = (
		directory.listFiles { file ->
			file.isFile && file.name.endsWith(".jsonl")
		} ?: emptyArray()
		)
		.sortedByDescending { it.lastModified() }

	private fun sendFile(exchange: HttpExchange, rawName: String) {
		val name = URLDecoder.decode(rawName, StandardCharsets.UTF_8)
		val file = File(directory, name)
		val valid = name.endsWith(".jsonl") &&
			!name.contains('/') &&
			!name.contains('\\') &&
			file.canonicalFile.parentFile == directory.canonicalFile &&
			file.isFile
		if (!valid) {
			send(exchange, 404, "text/plain; charset=utf-8", "not found".toByteArray())
			return
		}

		exchange.responseHeaders.add("Content-Type", "application/x-ndjson")
		exchange.responseHeaders.add(
			"Content-Disposition",
			"attachment; filename=\"${file.name}\"",
		)
		exchange.sendResponseHeaders(200, file.length())
		Files.copy(file.toPath(), exchange.responseBody)
	}

	private fun sendHtml(exchange: HttpExchange) {
		send(
			exchange,
			200,
			"text/html; charset=utf-8",
			HTML.toByteArray(StandardCharsets.UTF_8),
		)
	}

	private fun sendJson(exchange: HttpExchange, body: String) {
		exchange.responseHeaders.add("Cache-Control", "no-store")
		send(exchange, 200, "application/json; charset=utf-8", body.toByteArray(StandardCharsets.UTF_8))
	}

	private fun send(exchange: HttpExchange, status: Int, contentType: String, body: ByteArray) {
		exchange.responseHeaders.add("Content-Type", contentType)
		exchange.sendResponseHeaders(status, body.size.toLong())
		exchange.responseBody.use { it.write(body) }
	}

	private fun jsonBody(build: (JsonGenerator) -> Unit): String {
		val out = StringWriter()
		json.createGenerator(out).use { gen ->
			gen.writeStartObject()
			build(gen)
			gen.writeEndObject()
		}
		return out.toString()
	}

	private fun jsonArrayBody(build: (JsonGenerator) -> Unit): String {
		val out = StringWriter()
		json.createGenerator(out).use { gen ->
			gen.writeStartArray()
			build(gen)
			gen.writeEndArray()
		}
		return out.toString()
	}

	companion object {
		@Volatile
		private var active: LiveDataRecorder? = null

		/** Records a reset/calibration event when a recording is active. */
		fun event(tracker: Tracker, event: String) {
			active?.recordEvent(tracker, event)
		}

		private val HTML = """
			<!doctype html>
			<meta charset="utf-8">
			<title>SlimeVR live data</title>
			<style>
			body { font-family: system-ui, sans-serif; margin: 2rem; color: #ddd; background: #1b1b1f; }
			h1 { font-size: 1.4rem; }
			button { font: inherit; padding: .4rem .9rem; margin-right: .5rem; }
			li { margin: .3rem 0; }
			a { color: #9cc7ff; }
			#status { margin: 1rem 0; font-variant-numeric: tabular-nums; }
			</style>
			<h1>SlimeVR ライブデータ</h1>
			<p id="status">読み込み中...</p>
			<button onclick="post('/start')">記録開始</button>
			<button onclick="post('/stop')">記録停止</button>
			<h2 style="font-size:1.1rem">ファイル</h2>
			<ul id="files"></ul>
			<script>
			async function post(path) {
				await fetch(path, { method: 'POST' });
				refresh();
			}
			async function refresh() {
				const s = await (await fetch('/status')).json();
				const name = s.file ? s.file : 'なし';
				const secs = s.seconds.toFixed(1);
				document.getElementById('status').textContent =
					(s.recording ? '記録中' : '停止中') +
					' | ファイル: ' + name +
					' | ' + secs + ' 秒 / ' + s.samples + ' サンプル' +
					' | トラッカー: ' + s.trackers +
					' | フォルダ: ' + s.folder;
				const files = await (await fetch('/files')).json();
				document.getElementById('files').innerHTML = files.map(f =>
					'<li><a href="/files/' + encodeURIComponent(f.name) + '">' + f.name +
					'</a> (' + (f.bytes / 1024 / 1024).toFixed(1) + ' MB)</li>').join('');
			}
			refresh();
			setInterval(refresh, 1000);
			</script>
		""".trimIndent()
	}
}
