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

// Passive mode patrols a trail, so it is offered once the map has some.
const passiveTrailSelect = document.getElementById('passive-trail');

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

const personAlert = {
	overlay: document.getElementById('person-alert-overlay'),
	close: document.getElementById('person-alert-close'),
	photo: document.getElementById('person-alert-photo'),
	location: document.getElementById('person-alert-location')
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

// Top-down map: the terrain mapper's picture of the map, with the drone,
// trails, points of interest and the last ranger route drawn over it.
const mapElements = {
	frame: document.getElementById('map'),
	canvas: document.getElementById('map-canvas'),
	summary: document.getElementById('map-route-summary'),
	graphToggle: document.getElementById('btn-map-graph')
};

const mapUrls = {
	image: `${bridgeBaseUrl}/api/map.png`,
	state: `${bridgeBaseUrl}/api/map/state`,
	graph: `${bridgeBaseUrl}/api/map/graph`,
	route: `${bridgeBaseUrl}/api/route`
};

const mapColours = {
	trail: '#f08c1a',
	route: '#1f6fd6',
	graph: 'rgba(33, 61, 46, 0.22)',
	station: '#1f4fa0',
	drone: '#213d2e',
	poi: { red: '#d64545', green: '#3c9d4e', person: '#ffffff' }
};

const mapState = {
	image: null,
	info: null,
	trails: [],
	pois: [],
	route: null,
	graph: null,
	showGraph: false,
	pending: null,
	view: null
};

const localState = {
	x: null,
	y: null,
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
	localState.mode = payload.mode ?? localState.mode;
	localState.x = payload.x ?? localState.x;
	localState.y = payload.y ?? localState.y;
	if (Number.isFinite(Number(payload.battery))) {
		localState.battery = Number(payload.battery);
	}

	const distanceFromHome = Math.hypot(
		Number(localState.x ?? localState.latitude ?? 0),
		Number(localState.y ?? localState.longitude ?? 0)
	);
	if (distanceFromHome <= 0.5) {
		localState.battery = 100;
	} else {
		const currentBattery = Number.isFinite(localState.battery) ? Number(localState.battery) : 100;
		const drain = clamp(Number(localState.speed ?? 0) * 0.03, 0.02, 1.1);
		localState.battery = clamp(currentBattery - drain, 0, 100);
	}

	syncAltitudeFromTelemetry(localState.altitude);
	renderTelemetry(localState);
	drawMap();
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
			if (key === 'passive') {
				const trail = passiveTrailSelect ? passiveTrailSelect.value : '';
				publishCommand(trail ? `passive ${trail}` : 'passive');
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

// ------------------------------------------------------------ top-down map

// Where the map picture sits on the canvas: scale (canvas px per map cell)
// and offset, fitted to the frame keeping its aspect ratio.
function updateMapView() {
	const canvas = mapElements.canvas;
	const info = mapState.info;
	if (!canvas || !info) {
		mapState.view = null;
		return;
	}
	const ratio = window.devicePixelRatio || 1;
	const width = Math.round(canvas.clientWidth * ratio);
	const height = Math.round(canvas.clientHeight * ratio);
	if (canvas.width !== width || canvas.height !== height) {
		canvas.width = width;
		canvas.height = height;
	}
	const scale = Math.min(width / info.width, height / info.height);
	mapState.view = {
		scale,
		offsetX: (width - scale * info.width) / 2,
		offsetY: (height - scale * info.height) / 2,
		ratio
	};
}

// World metres -> canvas pixels. The picture has +y up, so row 0 is y_max.
function worldToCanvas(x, y) {
	const { info, view } = mapState;
	const col = (x - info.origin[0]) / info.resolution;
	const row = info.height - (y - info.origin[1]) / info.resolution;
	return [view.offsetX + col * view.scale, view.offsetY + row * view.scale];
}

function canvasToWorld(px, py) {
	const { info, view } = mapState;
	const col = (px - view.offsetX) / view.scale;
	const row = (py - view.offsetY) / view.scale;
	return [info.origin[0] + col * info.resolution, info.origin[1] + (info.height - row) * info.resolution];
}

function drawPolyline(ctx, points, colour, width) {
	if (!points || points.length < 2) {
		return;
	}
	ctx.beginPath();
	points.forEach(([x, y], i) => {
		const [px, py] = worldToCanvas(x, y);
		if (i === 0) {
			ctx.moveTo(px, py);
		} else {
			ctx.lineTo(px, py);
		}
	});
	ctx.strokeStyle = colour;
	ctx.lineWidth = width;
	ctx.lineJoin = 'round';
	ctx.lineCap = 'round';
	ctx.stroke();
}

function drawLabel(ctx, text, px, py, ratio) {
	ctx.font = `${Math.round(11 * ratio)}px "Segoe UI", sans-serif`;
	ctx.lineWidth = 3 * ratio;
	ctx.strokeStyle = 'rgba(250, 247, 240, 0.9)';
	ctx.strokeText(text, px, py);
	ctx.fillStyle = '#213d2e';
	ctx.fillText(text, px, py);
}

function drawMap() {
	const canvas = mapElements.canvas;
	if (!canvas || !mapState.image || !mapState.info) {
		return;
	}
	mapElements.frame.classList.add('terrain-active');
	updateMapView();
	const { view, info } = mapState;
	const ratio = view.ratio;
	const ctx = canvas.getContext('2d');
	ctx.clearRect(0, 0, canvas.width, canvas.height);
	ctx.imageSmoothingEnabled = false;
	ctx.drawImage(mapState.image, view.offsetX, view.offsetY, info.width * view.scale, info.height * view.scale);

	if (mapState.showGraph && mapState.graph) {
		ctx.beginPath();
		mapState.graph.edges.forEach(([i, j]) => {
			const [x0, y0] = worldToCanvas(mapState.graph.nodes[i][0], mapState.graph.nodes[i][1]);
			const [x1, y1] = worldToCanvas(mapState.graph.nodes[j][0], mapState.graph.nodes[j][1]);
			ctx.moveTo(x0, y0);
			ctx.lineTo(x1, y1);
		});
		ctx.strokeStyle = mapColours.graph;
		ctx.lineWidth = ratio;
		ctx.stroke();
	}

	mapState.trails.forEach((trail) => {
		drawPolyline(ctx, trail.points, mapColours.trail, 2 * ratio);
		if (trail.points.length) {
			const [x, y] = trail.points[Math.floor(trail.points.length / 2)];
			const [px, py] = worldToCanvas(x, y);
			drawLabel(ctx, trail.network ? `${trail.name} (${trail.network})` : trail.name, px + 6 * ratio, py - 6 * ratio, ratio);
		}
	});

	if (mapState.route && mapState.route.points.length) {
		drawPolyline(ctx, mapState.route.points, mapColours.route, 4 * ratio);
	}

	if (info.ranger_station) {
		const [px, py] = worldToCanvas(info.ranger_station[0], info.ranger_station[1]);
		const half = 6 * ratio;
		ctx.fillStyle = mapColours.station;
		ctx.fillRect(px - half, py - half, 2 * half, 2 * half);
		drawLabel(ctx, 'Station', px + 9 * ratio, py + 4 * ratio, ratio);
	}

	mapState.pois.forEach((poi) => {
		const [px, py] = worldToCanvas(poi.x, poi.y);
		ctx.beginPath();
		ctx.arc(px, py, 7 * ratio, 0, 2 * Math.PI);
		ctx.fillStyle = mapColours.poi[poi.type] || mapColours.poi.person;
		ctx.fill();
		ctx.lineWidth = 2 * ratio;
		ctx.strokeStyle = '#213d2e';
		ctx.stroke();
		drawLabel(ctx, poi.id, px + 10 * ratio, py + 4 * ratio, ratio);
	});

	if (mapState.pending) {
		const [px, py] = worldToCanvas(mapState.pending[0], mapState.pending[1]);
		ctx.beginPath();
		ctx.arc(px, py, 9 * ratio, 0, 2 * Math.PI);
		ctx.setLineDash([4 * ratio, 3 * ratio]);
		ctx.strokeStyle = mapColours.route;
		ctx.lineWidth = 2 * ratio;
		ctx.stroke();
		ctx.setLineDash([]);
	}

	// Drone: an arrow along its heading (degrees anticlockwise from +x)
	if (Number.isFinite(localState.x) && Number.isFinite(localState.y)) {
		const [px, py] = worldToCanvas(localState.x, localState.y);
		const yaw = toNumber(localState.heading, 0) * Math.PI / 180;
		ctx.save();
		ctx.translate(px, py);
		ctx.rotate(-yaw);
		ctx.beginPath();
		ctx.moveTo(12 * ratio, 0);
		ctx.lineTo(-8 * ratio, 7 * ratio);
		ctx.lineTo(-4 * ratio, 0);
		ctx.lineTo(-8 * ratio, -7 * ratio);
		ctx.closePath();
		ctx.fillStyle = mapColours.drone;
		ctx.fill();
		ctx.lineWidth = 1.5 * ratio;
		ctx.strokeStyle = '#ffffff';
		ctx.stroke();
		ctx.restore();
	}
}

function setMapSummary(text) {
	if (mapElements.summary) {
		mapElements.summary.textContent = text;
	}
}

function showPersonAlert(poi) {
	if (!personAlert.overlay || !personAlert.location || !personAlert.photo) {
		return;
	}

	const rgbFeed = document.getElementById('rgb-feed');
	const fallbackImage = 'data:image/svg+xml;charset=UTF-8,' + encodeURIComponent(`
		<svg xmlns="http://www.w3.org/2000/svg" width="200" height="200" viewBox="0 0 200 200">
			<rect width="200" height="200" fill="#dfe7e2"/>
			<circle cx="100" cy="78" r="32" fill="#aebdb4"/>
			<path d="M62 166c8-30 28-44 38-44s30 14 38 44" fill="#8aa58e"/>
			<rect x="16" y="16" width="168" height="168" rx="18" fill="none" stroke="#7a9783" stroke-width="4"/>
		</svg>
	`);

	const x = Number(poi?.x ?? 0);
	const y = Number(poi?.y ?? 0);
	personAlert.location.textContent = `Location: (${x.toFixed(1)}, ${y.toFixed(1)})`;
	personAlert.photo.src = rgbFeed && rgbFeed.currentSrc ? rgbFeed.currentSrc : fallbackImage;
	personAlert.overlay.classList.remove('hidden');
	personAlert.overlay.setAttribute('aria-hidden', 'false');
}

function hidePersonAlert() {
	if (!personAlert.overlay) {
		return;
	}
	personAlert.overlay.classList.add('hidden');
	personAlert.overlay.setAttribute('aria-hidden', 'true');
}

function handleDetectedPeople() {
	const people = (mapState.pois || []).filter((poi) => {
		const type = String(poi?.type || '').toLowerCase();
		const id = String(poi?.id || '').toLowerCase();
		return type.includes('person') || id.includes('person');
	});
	if (!people.length) {
		return;
	}

	const personKey = people.map((poi) => `${poi.id || 'person'}:${Number(poi.x).toFixed(1)}:${Number(poi.y).toFixed(1)}`).join('|');
	if (personAlert.lastSignature === personKey) {
		return;
	}
	personAlert.lastSignature = personKey;
	showPersonAlert(people[0]);
}

function describeRoute(route) {
	const info = route && route.info;
	if (!info) {
		return null;
	}
	const goal = `(${info.goal[0].toFixed(1)}, ${info.goal[1].toFixed(1)})`;
	if (!info.found) {
		return `No walking route from the station to ${goal}.`;
	}
	return `Route to ${goal}: ${info.length.toFixed(1)} m, ${(info.energy / 1000).toFixed(1)} kJ, `
		+ `${Math.round(100 * info.trail_share)}% on trail (${info.planner}, ${info.time_ms.toFixed(0)} ms).`;
}

// Trail names (T1 = longest) are re-assigned each time the trails are
// detected, so the list is rebuilt when they change, keeping the choice.
function updateTrailPicker() {
	if (!passiveTrailSelect) {
		return;
	}
	const trails = mapState.trails.filter((trail) => trail.name);
	const options = trails.length
		? [['', 'Nearest trail'], ...trails.map((trail) => [trail.name,
			`${trail.name}${trail.network ? ` (${trail.network})` : ''}, ${Number(trail.length).toFixed(1)} m`])]
		: [['', 'No trails detected yet']];
	const signature = JSON.stringify(options);
	if (passiveTrailSelect.dataset.signature !== signature) {
		const chosen = passiveTrailSelect.value;
		passiveTrailSelect.replaceChildren(...options.map(([value, label]) => new Option(label, value)));
		if (options.some(([value]) => value === chosen)) {
			passiveTrailSelect.value = chosen;
		}
		passiveTrailSelect.dataset.signature = signature;
	}
	passiveTrailSelect.disabled = !trails.length;
	if (commandButtons.passive) {
		commandButtons.passive.disabled = !trails.length;
		commandButtons.passive.title = trails.length ? 'Patrol the chosen trail at 0.8 m' : 'No trails detected yet: run Mapping first';
	}
}

async function refreshMapState() {
	try {
		const response = await fetch(mapUrls.state, { cache: 'no-store' });
		if (!response.ok) {
			return;
		}
		const payload = await response.json();
		mapState.info = payload.info;
		mapState.trails = payload.trails || [];
		updateTrailPicker();
		mapState.pois = payload.points_of_interest || [];
		handleDetectedPeople();
		const previous = mapState.route && mapState.route.info;
		mapState.route = payload.route;
		const current = mapState.route && mapState.route.info;
		if (current && JSON.stringify(current) !== JSON.stringify(previous) && !mapState.pending) {
			setMapSummary(describeRoute(mapState.route));
		}
		drawMap();
	} catch (error) {
		console.warn('Map state unavailable.', error);
	}
}

function refreshMapImage() {
	const image = new Image();
	image.onload = () => {
		mapState.image = image;
		drawMap();
	};
	image.src = `${mapUrls.image}?t=${Date.now()}`;
}

async function refreshMapGraph() {
	try {
		const response = await fetch(mapUrls.graph, { cache: 'no-store' });
		if (response.ok) {
			mapState.graph = await response.json();
			drawMap();
		}
	} catch (error) {
		console.warn('Route roadmap unavailable.', error);
	}
}

// A click on (or near) a point of interest plans a route to it; anywhere
// else plans a route to the clicked ground.
async function handleMapClick(event) {
	if (!mapState.view || !mapState.info || mapState.pending) {
		return;
	}
	const rect = mapElements.canvas.getBoundingClientRect();
	const ratio = mapState.view.ratio;
	const px = (event.clientX - rect.left) * ratio;
	const py = (event.clientY - rect.top) * ratio;

	let goal = canvasToWorld(px, py);
	let label = `(${goal[0].toFixed(1)}, ${goal[1].toFixed(1)})`;
	let nearest = 14 * ratio;
	mapState.pois.forEach((poi) => {
		const [qx, qy] = worldToCanvas(poi.x, poi.y);
		const distance = Math.hypot(qx - px, qy - py);
		if (distance < nearest) {
			nearest = distance;
			goal = [poi.x, poi.y];
			label = poi.id;
		}
	});

	mapState.pending = goal;
	setMapSummary(`Planning a route to ${label}...`);
	drawMap();
	try {
		const response = await fetch(mapUrls.route, {
			method: 'POST',
			headers: { 'Content-Type': 'application/json' },
			body: JSON.stringify({ x: goal[0], y: goal[1] })
		});
		const result = await response.json();
		mapState.pending = null;
		if (!result.ok) {
			setMapSummary(`No route to ${label}: ${result.error}.`);
		}
		await refreshMapState();
		if (result.ok) {
			setMapSummary(describeRoute(mapState.route));
		}
	} catch (error) {
		mapState.pending = null;
		setMapSummary('Route planner unavailable (is the bridge running?).');
		console.warn('Route request failed.', error);
	}
	drawMap();
}

function connectMap() {
	if (!mapElements.canvas) {
		return;
	}
	mapElements.canvas.addEventListener('click', handleMapClick);
	if (mapElements.graphToggle) {
		mapElements.graphToggle.addEventListener('click', () => {
			mapState.showGraph = !mapState.showGraph;
			mapElements.graphToggle.setAttribute('aria-pressed', String(mapState.showGraph));
			mapElements.graphToggle.textContent = mapState.showGraph ? 'Hide Roadmap' : 'Show Roadmap';
			if (mapState.showGraph) {
				refreshMapGraph();
			}
			drawMap();
		});
	}
	if (personAlert.close) {
		personAlert.close.addEventListener('click', hidePersonAlert);
	}
	if (personAlert.overlay) {
		personAlert.overlay.addEventListener('click', (event) => {
			if (event.target === personAlert.overlay) {
				hidePersonAlert();
			}
		});
	}
	window.addEventListener('resize', drawMap);
	refreshMapState();
	refreshMapImage();
	setInterval(refreshMapState, 1000);
	setInterval(refreshMapImage, 2000);
	setInterval(() => {
		if (mapState.showGraph) {
			refreshMapGraph();
		}
	}, 5000);
}

initMapView();
renderTelemetry(localState);
connectBridge();
connectMap();
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