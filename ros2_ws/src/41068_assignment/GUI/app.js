const telemetryValues = {
	coords: document.getElementById('val-coords'),
	altitude: document.getElementById('val-altitude'),
	heading: document.getElementById('val-heading'),
	speed: document.getElementById('val-speed'),
	mode: document.getElementById('val-mode'),
	battery: document.getElementById('stat-battery'),
	gps: document.getElementById('stat-gps')
};

const cameraFeeds = {
	rgb: {
		image: document.getElementById('rgb-feed'),
		placeholder: document.getElementById('rgb-placeholder'),
		status: document.getElementById('rgb-feed-status')
	},
	thermal: {
		image: document.getElementById('thermal-feed'),
		placeholder: document.getElementById('thermal-placeholder'),
		status: document.getElementById('thermal-feed-status')
	}
};

const commandButtons = {
	takeoff: document.getElementById('btn-takeoff'),
	rtl: document.getElementById('btn-rtl'),
	takeover: document.getElementById('btn-takeover'),
	mapping: document.getElementById('btn-mapping'),
	passive: document.getElementById('btn-passive'),
	active: document.getElementById('btn-active'),
	saveMap: document.getElementById('btn-save-map'),
	stop: document.getElementById('btn-stop')
};

const manualControls = {
	altitude: document.getElementById('altitude-slider'),
	altitudeValue: document.getElementById('altitude-value'),
	forward: document.getElementById('btn-forward'),
	back: document.getElementById('btn-back'),
	left: document.getElementById('btn-left'),
	right: document.getElementById('btn-right'),
	yawLeft: document.getElementById('btn-yaw-left'),
	yawRight: document.getElementById('btn-yaw-right')
};

const mapView = {
	frame: document.getElementById('map'),
	image: document.getElementById('map-image'),
	marker: document.getElementById('drone-marker')
};

const directionButtons = document.querySelectorAll('[data-direction]');
const statusIndicator = document.querySelector('.status-pill-online');
const bridgeBaseUrl = 'http://localhost:8081';
const telemetryUrl = `${bridgeBaseUrl}/api/telemetry`;
const commandUrl = `${bridgeBaseUrl}/api/command`;
const manualUrl = `${bridgeBaseUrl}/api/manual`;
const cameraFeedsUrl = {
	rgb: `${bridgeBaseUrl}/api/frame/rgb.jpg`,
	thermal: `${bridgeBaseUrl}/api/frame/thermal.jpg`
};

const cameraStreamsUrl = {
	rgb: `${bridgeBaseUrl}/stream/rgb.mjpg`,
	thermal: `${bridgeBaseUrl}/stream/thermal.mjpg`
};

const localState = {
	heading: 0,
	altitude: 0,
	speed: 0,
	battery: 0,
	latitude: 0,
	longitude: 0,
	mode: 'STANDBY'
};

const mapBounds = {
	xMin: -25,
	xMax: 25,
	yMin: -25,
	yMax: 25
};

let fallbackTimer = null;
let telemetryTimer = null;
let cameraTimer = null;
let manualTimer = null;
let isAltitudeSliderActive = false;

const manualState = {
	forward: 0,
	lateral: 0,
	yaw: 0,
	altitude: 0
};

function toNumber(value, fallback = 0) {
	const parsed = Number(value);
	return Number.isFinite(parsed) ? parsed : fallback;
}

function formatMetric(value, decimals = 1, suffix = '') {
	if (value === null || value === undefined || !Number.isFinite(Number(value))) {
		return 'N/A';
	}
	return `${Number(value).toFixed(decimals)}${suffix}`;
}

function clamp(value, min, max) {
	return Math.max(min, Math.min(max, value));
}

function initMapView() {
	if (!mapView.frame || !mapView.image) {
		return;
	}

	mapView.image.addEventListener('load', () => {
		mapView.frame.classList.add('map-active');
	});

	mapView.image.addEventListener('error', () => {
		mapView.frame.classList.remove('map-active');
	});

	if (mapView.image.complete && mapView.image.naturalWidth > 0) {
		mapView.frame.classList.add('map-active');
	}
}

function updateDroneMarker(x, y, heading = 0) {
	if (!mapView.marker) {
		return;
	}

	const xVal = Number(x);
	const yVal = Number(y);
	const headingVal = Number(heading);
	const normalizedHeading = Number.isFinite(headingVal) ? ((headingVal % 360) + 360) % 360 : 0;
	// Odometry heading is 0 at +X and increases CCW; the CSS arrow points up by
	// default in screen space, so convert to screen rotation.
	const screenRotation = 90 - normalizedHeading;
	mapView.marker.style.setProperty('--marker-rotation', `${screenRotation.toFixed(1)}deg`);

	if (!Number.isFinite(xVal) || !Number.isFinite(yVal)) {
		mapView.marker.style.display = 'none';
		return;
	}

	mapView.marker.style.display = 'block';
	const xNorm = clamp((xVal - mapBounds.xMin) / (mapBounds.xMax - mapBounds.xMin), 0, 1);
	const yNorm = clamp((yVal - mapBounds.yMin) / (mapBounds.yMax - mapBounds.yMin), 0, 1);

	mapView.marker.style.left = `${(xNorm * 100).toFixed(2)}%`;
	mapView.marker.style.top = `${((1 - yNorm) * 100).toFixed(2)}%`;
}

function setFeedActive(feedKey, active) {
	const feed = cameraFeeds[feedKey];
	if (!feed || !feed.image || !feed.image.parentElement) {
		return;
	}
	feed.image.parentElement.classList.toggle('feed-active', active);
	if (feed.status) {
		if (feedKey === 'rgb') {
			feed.status.textContent = active ? 'Live' : 'No signal';
		} else {
			feed.status.textContent = active ? 'Live' : 'Optional';
		}
	}
}

function configureCameraFeed(feedKey, streamUrl) {
	const feed = cameraFeeds[feedKey];
	if (!feed || !feed.image) {
		return;
	}

	feed.image.onload = () => setFeedActive(feedKey, true);
	feed.image.onerror = () => setFeedActive(feedKey, false);
	feed.image.src = `${streamUrl}?t=${Date.now()}`;
	setFeedActive(feedKey, true);
}

function configureCameraStream(feedKey, streamUrl, fallbackFrameUrl) {
	const feed = cameraFeeds[feedKey];
	if (!feed || !feed.image) {
		return;
	}

	feed.image.onload = () => setFeedActive(feedKey, true);
	feed.image.onerror = () => {
		setFeedActive(feedKey, false);
		configureCameraFeed(feedKey, fallbackFrameUrl);
	};

	feed.image.src = `${streamUrl}?t=${Date.now()}`;
}

function renderTelemetry(state) {
	const latitude = toNumber(state.latitude, 0);
	const longitude = toNumber(state.longitude, 0);
	updateDroneMarker(latitude, longitude, state.heading);
	telemetryValues.coords.textContent = Number.isFinite(latitude) && Number.isFinite(longitude)
		? `${latitude.toFixed(4)}, ${longitude.toFixed(4)}`
		: 'N/A';
	telemetryValues.altitude.textContent = formatMetric(state.altitude, 1, ' m');
	telemetryValues.heading.textContent = `${Math.round(toNumber(state.heading, 0)) % 360}°`;
	telemetryValues.speed.textContent = formatMetric(state.speed, 1, ' m/s');
	telemetryValues.mode.textContent = state.mode || 'STANDBY';
	telemetryValues.battery.textContent = Number.isFinite(Number(state.battery)) ? `${Math.round(toNumber(state.battery, 0))}%` : 'NA';
	if (telemetryValues.gps) {
		telemetryValues.gps.textContent = '3D FIX';
	}
}

function updateStatus(text) {
	if (statusIndicator) {
		statusIndicator.textContent = text;
	}
}

function applyTelemetry(payload) {
	localState.latitude = payload.latitude ?? localState.latitude;
	localState.longitude = payload.longitude ?? localState.longitude;
	localState.altitude = payload.altitude ?? localState.altitude;
	localState.heading = payload.heading ?? localState.heading;
	localState.speed = payload.speed ?? localState.speed;
	localState.battery = payload.battery ?? localState.battery;
	localState.mode = payload.mode ?? localState.mode;
	syncAltitudeFromTelemetry(localState.altitude);
	renderTelemetry(localState);
}

function tickFallbackTelemetry() {
	localState.heading = 0;
	localState.altitude = 0;
	localState.speed = 0;
	localState.latitude = 0;
	localState.longitude = 0;
	localState.battery = 0;
	localState.mode = 'STANDBY';
	renderTelemetry(localState);
}

function startFallbackMode() {
	if (fallbackTimer) {
		return;
	}
	updateStatus('Bridge Offline');
	fallbackTimer = setInterval(tickFallbackTelemetry, 1000);
}

function renderManualValues() {
	if (manualControls.altitudeValue) {
		manualControls.altitudeValue.textContent = `${Number(manualState.altitude).toFixed(1)} m`;
	}
}

function updateManualState() {
	manualState.altitude = Number(manualControls.altitude?.value ?? 0);
	renderManualValues();
}

function syncAltitudeFromTelemetry(altitude) {
	if (!manualControls.altitude || isAltitudeSliderActive) {
		return;
	}

	const parsedAltitude = Number(altitude);
	if (!Number.isFinite(parsedAltitude)) {
		return;
	}

	const clampedAltitude = Math.min(16, Math.max(0, parsedAltitude));
	manualState.altitude = clampedAltitude;
	manualControls.altitude.value = clampedAltitude.toFixed(1);
	renderManualValues();
}

function setManualVector(forward, lateral, yaw = 0) {
	manualState.forward = forward;
	manualState.lateral = lateral;
	manualState.yaw = yaw;
	publishManualControl();
}

function resetManualVector() {
	setManualVector(0, 0, 0);
}

async function refreshTelemetry() {
	try {
		const response = await fetch(telemetryUrl, { cache: 'no-store' });
		if (!response.ok) {
			throw new Error(`Telemetry request failed: ${response.status}`);
		}
		const payload = await response.json();
		updateStatus('Bridge Connected');
		if (fallbackTimer) {
			clearInterval(fallbackTimer);
			fallbackTimer = null;
		}
		applyTelemetry(payload);
		if (payload.mode) {
			localState.mode = payload.mode;
		}
	} catch (error) {
		console.warn('Telemetry bridge unavailable, using placeholder data.', error);
		updateStatus('Bridge Offline');
		startFallbackMode();
	}
}

function connectBridge() {
	configureCameraStream('rgb', cameraStreamsUrl.rgb, cameraFeedsUrl.rgb);
	configureCameraStream('thermal', cameraStreamsUrl.thermal, cameraFeedsUrl.thermal);
	refreshTelemetry();
	telemetryTimer = setInterval(refreshTelemetry, 100);
	updateManualState();
}

async function publishCommand(command) {
	const labels = {
		takeoff: 'TAKE OFF',
		take_over: 'TAKE OVER',
		return_home: 'RETURN HOME',
		stop: 'STOP',
		save_map: 'SAVE MAP',
		mapping: 'MAPPING',
		passive_t3: 'PASSIVE T3',
		active: 'ACTIVE'
	};
	localState.mode = labels[command] || command.toUpperCase();
	renderTelemetry(localState);

	try {
		await fetch(commandUrl, {
			method: 'POST',
			headers: {
				'Content-Type': 'application/json'
			},
			body: JSON.stringify({ command })
		});
	} catch (error) {
		console.warn('Command bridge unavailable.', error);
	}
}

async function publishManualControl() {
	try {
		await fetch(manualUrl, {
			method: 'POST',
			headers: {
				'Content-Type': 'application/json'
			},
			body: JSON.stringify(manualState)
		});
	} catch (error) {
		console.warn('Manual control bridge unavailable.', error);
	}
}

function sendTakeoff() {
	localState.mode = 'TAKE OFF';
	renderTelemetry(localState);
	publishCommand('takeoff');
}

function bindCommandButtons() {
	Object.entries(commandButtons).forEach(([key, button]) => {
		if (!button) {
			return;
		}
		button.addEventListener('click', () => {
			if (key === 'takeoff') {
				sendTakeoff();
				return;
			}
			if (key === 'takeover') {
				publishCommand('take_over');
				return;
			}
			if (key === 'rtl') {
				publishCommand('return_home');
				return;
			}
			const commandName = button.dataset.command || key;
			publishCommand(commandName);
		});
	});
}

function bindMomentaryManualButton(button, vector) {
	if (!button) {
		return;
	}

	let engaged = false;

	const activate = (event) => {
		if (event) {
			event.preventDefault();
		}
		engaged = true;
		setManualVector(vector.forward, vector.lateral, vector.yaw);
	};

	const release = () => {
		if (!engaged) {
			return;
		}
		engaged = false;
		resetManualVector();
	};

	button.addEventListener('pointerdown', activate);
	button.addEventListener('pointerup', release);
	button.addEventListener('pointerleave', release);
	button.addEventListener('pointercancel', release);
	button.addEventListener('mousedown', activate);
	button.addEventListener('mouseup', release);
	button.addEventListener('mouseleave', release);
	button.addEventListener('touchstart', activate, { passive: false });
	button.addEventListener('touchend', release);
	button.addEventListener('touchcancel', release);
}

initMapView();
renderTelemetry(localState);
connectBridge();
bindCommandButtons();

manualControls.altitude.addEventListener('pointerdown', () => {
	isAltitudeSliderActive = true;
});

manualControls.altitude.addEventListener('pointerup', () => {
	isAltitudeSliderActive = false;
});

manualControls.altitude.addEventListener('blur', () => {
	isAltitudeSliderActive = false;
});

manualControls.altitude.addEventListener('input', () => {
	isAltitudeSliderActive = true;
	updateManualState();
	publishManualControl();
});

manualControls.altitude.addEventListener('change', () => {
	isAltitudeSliderActive = false;
});

bindMomentaryManualButton(manualControls.forward, { forward: 1, lateral: 0, yaw: 0 });
bindMomentaryManualButton(manualControls.back, { forward: -1, lateral: 0, yaw: 0 });
bindMomentaryManualButton(manualControls.left, { forward: 0, lateral: 1, yaw: 0 });
bindMomentaryManualButton(manualControls.right, { forward: 0, lateral: -1, yaw: 0 });
bindMomentaryManualButton(manualControls.yawLeft, { forward: 0, lateral: 0, yaw: 1 });
bindMomentaryManualButton(manualControls.yawRight, { forward: 0, lateral: 0, yaw: -1 });

manualTimer = setInterval(() => {
	if (Math.abs(manualState.forward) > 0.001 || Math.abs(manualState.lateral) > 0.001 || Math.abs(manualState.yaw) > 0.001) {
		publishManualControl();
	}
}, 300);

console.log('ROS bridge dashboard initialized.');