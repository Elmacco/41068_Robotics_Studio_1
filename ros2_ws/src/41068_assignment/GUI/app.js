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
	takeover: document.getElementById('btn-takeover')
};

const manualControls = {
	forward: document.getElementById('forward-slider'),
	lateral: document.getElementById('lateral-slider'),
	forwardValue: document.getElementById('forward-value'),
	lateralValue: document.getElementById('lateral-value')
};

const statusIndicator = document.querySelector('.status-pill-online');
const bridgeBaseUrl = 'http://localhost:8081';
const telemetryUrl = `${bridgeBaseUrl}/api/telemetry`;
const commandUrl = `${bridgeBaseUrl}/api/command`;
const manualUrl = `${bridgeBaseUrl}/api/manual`;
const cameraFeedsUrl = {
	rgb: `${bridgeBaseUrl}/api/frame/rgb.jpg`,
	thermal: `${bridgeBaseUrl}/api/frame/thermal.jpg`
};

const localState = {
	heading: 84,
	altitude: 18.4,
	speed: 4.8,
	battery: 98,
	latitude: -33.8688,
	longitude: 151.2093,
	mode: 'STANDBY'
};

let ros = null;
let fallbackTimer = null;
let telemetryTimer = null;
let cameraTimer = null;
let manualTimer = null;

const manualState = {
	forward: 0,
	lateral: 0
};

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

function renderTelemetry(state) {
	telemetryValues.coords.textContent = `${state.latitude.toFixed(4)}, ${state.longitude.toFixed(4)}`;
	telemetryValues.altitude.textContent = `${state.altitude.toFixed(1)} m`;
	telemetryValues.heading.textContent = `${Math.round(state.heading) % 360}°`;
	telemetryValues.speed.textContent = `${state.speed.toFixed(1)} m/s`;
	telemetryValues.mode.textContent = state.mode;
	telemetryValues.battery.textContent = `${Math.round(state.battery)}%`;
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
	renderTelemetry(localState);
}

function tickFallbackTelemetry() {
	localState.heading = (localState.heading + 3) % 360;
	localState.altitude = 18.4 + Math.sin(Date.now() / 2600) * 0.8;
	localState.speed = 4.2 + Math.cos(Date.now() / 2100) * 0.5;
	localState.latitude += 0.00002;
	localState.longitude += 0.00003;
	localState.battery = Math.max(82, localState.battery - 0.01);
	localState.mode = 'HOVER';
	renderTelemetry(localState);
}

function startFallbackMode() {
	if (fallbackTimer) {
		return;
	}
	updateStatus('Local Mock Active');
	fallbackTimer = setInterval(tickFallbackTelemetry, 1000);
}

function renderManualValues() {
	if (manualControls.forwardValue) {
		manualControls.forwardValue.textContent = `${Number(manualState.forward).toFixed(1)} m/s`;
	}
	if (manualControls.lateralValue) {
		manualControls.lateralValue.textContent = `${Number(manualState.lateral).toFixed(1)} m/s`;
	}
}

function updateManualState() {
	manualState.forward = Number(manualControls.forward?.value ?? 0);
	manualState.lateral = Number(manualControls.lateral?.value ?? 0);
	renderManualValues();
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
		console.warn('Telemetry bridge unavailable, using local fallback.', error);
		updateStatus('Bridge Offline');
		startFallbackMode();
	}
}

function connectBridge() {
	configureCameraFeed('rgb', cameraFeedsUrl.rgb);
	configureCameraFeed('thermal', cameraFeedsUrl.thermal);
	refreshTelemetry();
	telemetryTimer = setInterval(refreshTelemetry, 1000);
	cameraTimer = setInterval(() => {
		configureCameraFeed('rgb', cameraFeedsUrl.rgb);
	}, 400);
	renderManualValues();
}

async function publishCommand(command) {
	localState.mode = command === 'return_home' ? 'RETURN HOME' : 'TAKE OVER';
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
	updateManualState();
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

renderTelemetry(localState);
connectBridge();

commandButtons.takeoff.addEventListener('click', () => {
	sendTakeoff();
});

commandButtons.rtl.addEventListener('click', () => {
	publishCommand('return_home');
});

commandButtons.takeover.addEventListener('click', () => {
	publishCommand('take_over');
});

manualControls.forward.addEventListener('input', () => {
	publishManualControl();
});

manualControls.lateral.addEventListener('input', () => {
	publishManualControl();
});

manualTimer = setInterval(() => {
	if (Math.abs(manualState.forward) > 0.001 || Math.abs(manualState.lateral) > 0.001) {
		publishManualControl();
	}
}, 300);

console.log('ROS bridge dashboard initialized.');