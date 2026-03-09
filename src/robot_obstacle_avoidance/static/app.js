const COLORS = {
    astar: "#0f766e",
    apf: "#d97706",
    dwa: "#1d4ed8",
};

const state = {
    meta: null,
    scenario: null,
    results: [],
    compareAll: false,
    frame: 0,
    isPlaying: true,
    speed: 1,
    lastTick: 0,
    previewTimeSeconds: 0,
    customObstaclesByScenario: {},
    runtimeObstacleEventsByScenario: {},
    runToken: 0,
};

const elements = {
    scenarioSelect: document.getElementById("scenario-select"),
    modeSelect: document.getElementById("mode-select"),
    algorithmField: document.getElementById("algorithm-field"),
    algorithmSelect: document.getElementById("algorithm-select"),
    dtInput: document.getElementById("dt-input"),
    runButton: document.getElementById("run-button"),
    playButton: document.getElementById("play-button"),
    frameSlider: document.getElementById("frame-slider"),
    speedSlider: document.getElementById("speed-slider"),
    frameReadout: document.getElementById("frame-readout"),
    speedReadout: document.getElementById("speed-readout"),
    statusBanner: document.getElementById("status-banner"),
    metricsGrid: document.getElementById("metrics-grid"),
    legendRow: document.getElementById("legend-row"),
    canvas: document.getElementById("scene-canvas"),
    obstacleTypeSelect: document.getElementById("obstacle-type-select"),
    circleRadiusField: document.getElementById("circle-radius-field"),
    obstacleRadiusInput: document.getElementById("obstacle-radius-input"),
    obstacleWidthInput: document.getElementById("obstacle-width-input"),
    obstacleHeightInput: document.getElementById("obstacle-height-input"),
    undoObstacleButton: document.getElementById("undo-obstacle-button"),
    clearObstaclesButton: document.getElementById("clear-obstacles-button"),
    obstacleList: document.getElementById("obstacle-list"),
};

const ctx = elements.canvas.getContext("2d");

async function bootstrap() {
    bindEvents();
    await loadMeta();
    applyScenarioSelection(false);
    await runCurrentSelection();
    window.requestAnimationFrame(animationLoop);
}

function bindEvents() {
    elements.modeSelect.addEventListener("change", async () => {
        const compare = elements.modeSelect.value === "compare";
        state.compareAll = compare;
        elements.algorithmField.style.opacity = compare ? "0.45" : "1";
        elements.algorithmSelect.disabled = compare;
        resetDisplayedResults();
        renderLegend();
        setStatus(compare ? "Compare-all mode selected. Recomputing trajectories..." : "Single-algorithm mode selected. Recomputing trajectory...");
        await runCurrentSelection();
    });

    elements.scenarioSelect.addEventListener("change", async () => {
        applyScenarioSelection();
        await runCurrentSelection();
    });

    elements.algorithmSelect.addEventListener("change", async () => {
        if (elements.modeSelect.value === "compare") {
            return;
        }
        resetDisplayedResults();
        renderLegend();
        setStatus(`Algorithm changed to ${elements.algorithmSelect.value.toUpperCase()}. Recomputing trajectory...`);
        await runCurrentSelection();
    });

    elements.dtInput.addEventListener("change", async () => {
        resetDisplayedResults();
        setStatus("Time-step updated. Recomputing trajectory...");
        await runCurrentSelection();
    });

    elements.runButton.addEventListener("click", async () => {
        await runCurrentSelection();
    });

    elements.playButton.addEventListener("click", () => {
        state.isPlaying = !state.isPlaying;
        elements.playButton.textContent = state.isPlaying ? "Pause" : "Play";
    });

    elements.frameSlider.addEventListener("input", (event) => {
        state.frame = Number(event.target.value);
        state.isPlaying = false;
        elements.playButton.textContent = "Play";
        renderScene();
        updateFrameReadout();
    });

    elements.speedSlider.addEventListener("input", (event) => {
        state.speed = Number(event.target.value);
        elements.speedReadout.textContent = `${state.speed.toFixed(2)}x`;
    });

    elements.obstacleTypeSelect.addEventListener("change", () => {
        syncObstacleInputs();
    });

    elements.undoObstacleButton.addEventListener("click", async () => {
        const runtimeObstacleEvents = [...getRuntimeObstacleEvents()];
        let removedRuntimeEvent = false;
        if (runtimeObstacleEvents.length > 0) {
            runtimeObstacleEvents.pop();
            setRuntimeObstacleEvents(runtimeObstacleEvents);
            removedRuntimeEvent = true;
            if (state.results.length > 0) {
                await rerunLiveScenario("Removed the most recent live obstacle insertion.");
                return;
            }
        }

        const customObstacles = [...getCustomObstacles()];
        if (customObstacles.length === 0 && !removedRuntimeEvent) {
            return;
        }
        if (customObstacles.length > 0) {
            customObstacles.pop();
            setCustomObstacles(customObstacles);
        }
        markSimulationStale("Removed the most recent placed obstacle.");
    });

    elements.clearObstaclesButton.addEventListener("click", async () => {
        if (getCustomObstacles().length === 0 && getRuntimeObstacleEvents().length === 0) {
            return;
        }
        setCustomObstacles([]);
        setRuntimeObstacleEvents([]);
        if (state.results.length > 0) {
            await rerunLiveScenario("Cleared all placed obstacles for the current scenario.");
            return;
        }
        markSimulationStale("Cleared all placed obstacles for the current scenario.");
    });

    elements.obstacleList.addEventListener("click", async (event) => {
        const button = event.target.closest("button[data-index]");
        if (!button) {
            return;
        }
        const obstacleIndex = Number(button.dataset.index);
        if (button.dataset.source === "runtime") {
            const runtimeObstacleEvents = [...getRuntimeObstacleEvents()];
            runtimeObstacleEvents.splice(obstacleIndex, 1);
            setRuntimeObstacleEvents(runtimeObstacleEvents);
            if (state.results.length > 0) {
                await rerunLiveScenario("Removed a live obstacle insertion.");
                return;
            }
            markSimulationStale("Removed a scheduled live obstacle insertion.");
            return;
        }

        const customObstacles = [...getCustomObstacles()];
        customObstacles.splice(obstacleIndex, 1);
        setCustomObstacles(customObstacles);
        markSimulationStale("Removed a custom obstacle.");
    });

    elements.canvas.addEventListener("click", (event) => {
        void handleCanvasPlacement(event);
    });
}

async function loadMeta() {
    const response = await fetch("/api/meta");
    const payload = await response.json();
    state.meta = payload;

    for (const scenario of payload.scenarios) {
        const option = document.createElement("option");
        option.value = scenario.name;
        option.textContent = scenario.name.replaceAll("_", " ");
        elements.scenarioSelect.appendChild(option);
    }

    for (const algorithm of payload.algorithms) {
        const option = document.createElement("option");
        option.value = algorithm;
        option.textContent = algorithm.toUpperCase();
        elements.algorithmSelect.appendChild(option);
    }

    elements.scenarioSelect.value = payload.defaults.scenario;
    elements.algorithmSelect.value = payload.defaults.algorithm;
    elements.dtInput.value = payload.defaults.dt.toFixed(2);
    elements.obstacleRadiusInput.value = payload.obstacle_editor.default_radius.toFixed(2);
    elements.obstacleWidthInput.value = payload.obstacle_editor.default_width.toFixed(1);
    elements.obstacleHeightInput.value = payload.obstacle_editor.default_height.toFixed(1);
    state.speed = Number(elements.speedSlider.value);
    elements.speedReadout.textContent = `${state.speed.toFixed(2)}x`;
    syncObstacleInputs();
}

function applyScenarioSelection(showStatus = true) {
    ensureScenarioStores();
    refreshPreviewScenario();
    resetDisplayedResults();
    renderObstacleList();
    renderLegend();
    renderMetrics();
    renderScene();
    if (showStatus) {
        setStatus("Scenario preview updated. Custom obstacles are preserved per scenario.");
    }
}

async function runCurrentSelection(options = {}) {
    const runToken = ++state.runToken;
    const scenario = elements.scenarioSelect.value;
    const algorithm = elements.algorithmSelect.value;
    const dt = Number(elements.dtInput.value);
    const compare = elements.modeSelect.value === "compare";
    const requestedStartFrame = Number.isFinite(options.startFrame) ? Number(options.startFrame) : 0;

    setStatus(compare ? "Running multi-algorithm comparison..." : `Running ${algorithm.toUpperCase()}...`);
    elements.runButton.disabled = true;

    try {
        const endpoint = compare ? "/api/compare" : "/api/simulate";
        const response = await fetch(endpoint, {
            method: "POST",
            headers: {
                "Content-Type": "application/json",
            },
            body: JSON.stringify({
                algorithm,
                scenario,
                dt,
                custom_obstacles: getCustomObstacles(),
                runtime_obstacle_events: getRuntimeObstacleEvents(),
            }),
        });
        const payload = await response.json();

        if (!response.ok) {
            throw new Error(payload.error || "Simulation request failed.");
        }

        if (runToken !== state.runToken) {
            return;
        }

        state.compareAll = compare;
        state.scenario = payload.scenario;
        state.results = compare ? payload.results : [payload.result];
        state.frame = 0;
        state.isPlaying = true;
        state.lastTick = 0;
        elements.playButton.textContent = "Pause";
        syncFrameSlider();
        state.frame = clampValue(requestedStartFrame, 0, Number(elements.frameSlider.max));
        elements.frameSlider.value = String(state.frame);
        updateFrameReadout();
        renderLegend();
        renderMetrics();
        renderScene();
        setStatus(buildSummaryMessage());
        return payload;
    } catch (error) {
        if (runToken === state.runToken) {
            setStatus(error.message, true);
        }
        return null;
    } finally {
        if (runToken === state.runToken) {
            elements.runButton.disabled = false;
        }
    }
}

function buildSummaryMessage() {
    if (state.results.length === 0) {
        return "No results loaded.";
    }

    const customObstacleCount = getCustomObstacles().length;
    const customSuffix = customObstacleCount > 0 ? ` with ${customObstacleCount} custom obstacle${customObstacleCount === 1 ? "" : "s"}` : "";
    const dynamicObstacleCount = (state.scenario?.dynamic_obstacles || []).length;
    const dynamicSuffix = dynamicObstacleCount > 0 ? ` and ${dynamicObstacleCount} dynamic obstacle${dynamicObstacleCount === 1 ? "" : "s"}` : "";
    const runtimeEventCount = (state.scenario?.runtime_obstacle_events || []).length;
    const runtimeSuffix = runtimeEventCount > 0 ? ` plus ${runtimeEventCount} live insertion${runtimeEventCount === 1 ? "" : "s"}` : "";
    const completed = state.results.filter((result) => result.success).length;
    if (state.compareAll) {
        const successfulResults = state.results.filter((result) => result.success);
        const bestResult = successfulResults.sort((left, right) => left.travel_time - right.travel_time)[0];
        const winnerSuffix = bestResult ? ` Best performer: ${bestResult.algorithm.toUpperCase()}.` : "";
        return `Comparison complete${customSuffix}${dynamicSuffix}${runtimeSuffix}. ${completed}/${state.results.length} algorithms reached the goal.${winnerSuffix}`;
    }

    const result = state.results[0];
    return `${result.algorithm.toUpperCase()} complete${customSuffix}${dynamicSuffix}${runtimeSuffix}. Success=${result.success}, path=${result.path_length.toFixed(2)} m, clearance=${result.min_clearance.toFixed(2)} m.`;
}

function syncFrameSlider() {
    const maxLength = state.results.length > 0
        ? Math.max(...state.results.map((result) => result.trajectory.length), 1)
        : 1;
    elements.frameSlider.max = String(Math.max(0, maxLength - 1));
    elements.frameSlider.value = String(state.frame);
    updateFrameReadout();
}

function renderLegend() {
    elements.legendRow.innerHTML = "";
    if (!state.meta) {
        return;
    }

    const algorithms = state.results.length > 0
        ? state.results.map((result) => result.algorithm)
        : (elements.modeSelect.value === "compare" ? state.meta.algorithms : [elements.algorithmSelect.value]);

    for (const algorithm of algorithms) {
        const item = document.createElement("div");
        item.className = "legend-item";
        item.innerHTML = `<span class="legend-swatch" style="background:${colorFor(algorithm)}"></span><span>${algorithm.toUpperCase()}</span>`;
        elements.legendRow.appendChild(item);
    }
}

function renderMetrics() {
    elements.metricsGrid.innerHTML = "";
    if (state.results.length === 0) {
        elements.metricsGrid.innerHTML = `
            <div class="metrics-empty">
                Run a simulation to populate the performance dashboard. Custom obstacles update the preview immediately and affect the next run.
            </div>
        `;
        return;
    }

    for (const result of state.results) {
        const card = document.createElement("article");
        card.className = "metric-card";
        card.style.background = buildMetricBackground(result.algorithm);
        card.innerHTML = `
            <h3>${result.algorithm.toUpperCase()} ${result.success ? "success" : "incomplete"}</h3>
            <div class="metric-list">
                <div><span>Travel time</span><strong>${result.travel_time.toFixed(2)} s</strong></div>
                <div><span>Path length</span><strong>${result.path_length.toFixed(2)} m</strong></div>
                <div><span>Minimum clearance</span><strong>${result.min_clearance.toFixed(2)} m</strong></div>
                <div><span>Final goal error</span><strong>${result.final_distance_to_goal.toFixed(2)} m</strong></div>
                <div><span>Smoothness</span><strong>${result.smoothness.toFixed(2)} rad</strong></div>
                <div><span>Steps</span><strong>${result.steps}</strong></div>
            </div>
        `;
        elements.metricsGrid.appendChild(card);
    }
}

function renderObstacleList() {
    const customObstacles = getCustomObstacles();
    const runtimeObstacleEvents = getRuntimeObstacleEvents();
    elements.undoObstacleButton.disabled = customObstacles.length === 0 && runtimeObstacleEvents.length === 0;
    elements.clearObstaclesButton.disabled = customObstacles.length === 0 && runtimeObstacleEvents.length === 0;

    if (customObstacles.length === 0 && runtimeObstacleEvents.length === 0) {
        elements.obstacleList.innerHTML = `
            <div class="obstacle-empty">
                No placed obstacles for this scenario yet. Click before running to place a static obstacle, or click during playback to inject a live obstacle at the current time.
            </div>
        `;
        return;
    }

    const customItems = customObstacles.map((obstacle, index) => {
        const description = obstacle.kind === "circle"
            ? `center=(${obstacle.x.toFixed(2)}, ${obstacle.y.toFixed(2)}), radius=${obstacle.radius.toFixed(2)}`
            : `center=(${obstacle.x.toFixed(2)}, ${obstacle.y.toFixed(2)}), size=${obstacle.width.toFixed(2)}×${obstacle.height.toFixed(2)}`;

        return `
            <div class="obstacle-item">
                <div class="obstacle-copy">
                    <strong>${index + 1}. Static ${obstacle.kind === "circle" ? "circle" : "rectangle block"}</strong>
                    <span>${description}</span>
                </div>
                <button class="mini-button" type="button" data-index="${index}" data-source="custom">Remove</button>
            </div>
        `;
    });

    const runtimeItems = runtimeObstacleEvents.map((event, index) => {
        const obstacle = event.obstacle;
        const description = obstacle.kind === "circle"
            ? `t=${event.activate_time.toFixed(2)} s, center=(${obstacle.x.toFixed(2)}, ${obstacle.y.toFixed(2)}), radius=${obstacle.radius.toFixed(2)}`
            : `t=${event.activate_time.toFixed(2)} s, center=(${obstacle.x.toFixed(2)}, ${obstacle.y.toFixed(2)}), size=${obstacle.width.toFixed(2)}×${obstacle.height.toFixed(2)}`;

        return `
            <div class="obstacle-item">
                <div class="obstacle-copy">
                    <strong>${index + 1}. Live ${obstacle.kind === "circle" ? "circle" : "rectangle block"}</strong>
                    <span>${description}</span>
                </div>
                <button class="mini-button" type="button" data-index="${index}" data-source="runtime">Remove</button>
            </div>
        `;
    });

    elements.obstacleList.innerHTML = [...customItems, ...runtimeItems].join("");
}

function syncObstacleInputs() {
    const isCircle = elements.obstacleTypeSelect.value === "circle";
    elements.circleRadiusField.style.display = isCircle ? "flex" : "none";
    document.querySelectorAll(".rectangle-dimension-field").forEach((element) => {
        element.style.display = isCircle ? "none" : "flex";
    });
}

function animationLoop(timestamp) {
    state.previewTimeSeconds = timestamp / 1000;

    if (!state.lastTick) {
        state.lastTick = timestamp;
    }

    const elapsed = timestamp - state.lastTick;
    const stepInterval = 70 / Math.max(state.speed, 0.01);
    if (state.isPlaying && state.results.length > 0 && elapsed >= stepInterval) {
        const maxFrame = Number(elements.frameSlider.max);
        state.frame = state.frame < maxFrame ? state.frame + 1 : 0;
        elements.frameSlider.value = String(state.frame);
        updateFrameReadout();
        renderScene();
        state.lastTick = timestamp;
    } else if (state.results.length === 0 && (state.scenario?.dynamic_obstacles?.length || 0) > 0) {
        renderScene();
    }

    window.requestAnimationFrame(animationLoop);
}

function renderScene() {
    const scenario = state.scenario || getSelectedScenarioDefinition();
    if (!scenario) {
        return;
    }

    const { width, height } = elements.canvas;
    ctx.clearRect(0, 0, width, height);
    drawBackdrop(width, height);

    const projection = getCanvasProjection(scenario);
    const project = (x, y) => ({
        x: projection.padding + x * projection.scale,
        y: projection.height - projection.padding - y * projection.scale,
    });

    drawGrid(scenario, project);
    drawBoundary(scenario, project);
    drawObstacles(scenario, project, projection.scale);
    drawRuntimeObstacles(getCurrentRuntimeObstacleSnapshots(scenario), project, projection.scale);
    drawDynamicObstacles(getCurrentDynamicObstacleSnapshots(scenario), project, projection.scale);
    drawGoalMarkers(scenario, project, projection.scale);

    for (const result of state.results) {
        drawTrajectory(result, project);
        drawRobot(result, project, projection.scale, scenario.robot_radius);
    }
}

function getCanvasProjection(scenario) {
    const width = elements.canvas.width;
    const height = elements.canvas.height;
    const padding = 56;
    const scale = Math.min(
        (width - padding * 2) / scenario.width,
        (height - padding * 2) / scenario.height,
    );

    return { width, height, padding, scale };
}

function drawBackdrop(width, height) {
    const gradient = ctx.createLinearGradient(0, 0, width, height);
    gradient.addColorStop(0, "rgba(255,255,255,0.98)");
    gradient.addColorStop(1, "rgba(238,242,255,0.92)");
    ctx.fillStyle = gradient;
    ctx.fillRect(0, 0, width, height);
}

function drawGrid(scenario, project) {
    ctx.save();
    ctx.strokeStyle = "rgba(71, 85, 105, 0.12)";
    ctx.lineWidth = 1;
    for (let x = 0; x <= scenario.width; x += 1) {
        const p1 = project(x, 0);
        const p2 = project(x, scenario.height);
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
    }
    for (let y = 0; y <= scenario.height; y += 1) {
        const p1 = project(0, y);
        const p2 = project(scenario.width, y);
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
    }
    ctx.restore();
}

function drawBoundary(scenario, project) {
    const topLeft = project(0, scenario.height);
    const bottomRight = project(scenario.width, 0);
    ctx.save();
    ctx.strokeStyle = "#1f2937";
    ctx.lineWidth = 3;
    ctx.strokeRect(topLeft.x, topLeft.y, bottomRight.x - topLeft.x, bottomRight.y - topLeft.y);
    ctx.restore();
}

function drawObstacles(scenario, project, scale) {
    const baseObstacleCount = getSelectedScenarioDefinition()?.obstacles.length ?? scenario.obstacles.length;
    scenario.obstacles.forEach((obstacle, index) => {
        const isCustom = index >= baseObstacleCount;
        if (obstacle.kind === "circle") {
            drawCircularObstacle(obstacle, project, scale, isCustom);
        } else {
            drawRectangularObstacle(obstacle, project, scale, isCustom);
        }
    });
}

function drawRuntimeObstacles(obstacles, project, scale) {
    obstacles.forEach((obstacle) => {
        if (obstacle.kind === "circle") {
            drawRuntimeCircularObstacle(obstacle, project, scale);
        } else {
            drawRuntimeRectangularObstacle(obstacle, project, scale);
        }
    });
}

function drawDynamicObstacles(obstacles, project, scale) {
    obstacles.forEach((obstacle) => {
        const center = project(obstacle.x, obstacle.y);
        const radius = obstacle.radius * scale;
        const halo = ctx.createRadialGradient(center.x, center.y, radius * 0.1, center.x, center.y, radius * 1.55);
        halo.addColorStop(0, "rgba(56, 189, 248, 0.95)");
        halo.addColorStop(0.6, "rgba(37, 99, 235, 0.35)");
        halo.addColorStop(1, "rgba(37, 99, 235, 0.04)");

        ctx.save();
        ctx.beginPath();
        ctx.fillStyle = halo;
        ctx.arc(center.x, center.y, radius * 1.55, 0, Math.PI * 2);
        ctx.fill();

        ctx.beginPath();
        ctx.fillStyle = "rgba(37, 99, 235, 0.88)";
        ctx.arc(center.x, center.y, radius, 0, Math.PI * 2);
        ctx.fill();

        ctx.beginPath();
        ctx.setLineDash([6, 5]);
        ctx.lineWidth = 2.2;
        ctx.strokeStyle = "rgba(191, 219, 254, 0.95)";
        ctx.arc(center.x, center.y, radius * 1.08, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);

        if (obstacle.label) {
            ctx.fillStyle = "#0f172a";
            ctx.font = '600 12px "Space Grotesk", sans-serif';
            ctx.fillText(obstacle.label, center.x + radius + 6, center.y - radius - 4);
        }
        ctx.restore();
    });
}

function drawRuntimeCircularObstacle(obstacle, project, scale) {
    const center = project(obstacle.x, obstacle.y);
    const radius = obstacle.radius * scale;
    const fillGradient = ctx.createRadialGradient(center.x - radius * 0.2, center.y - radius * 0.2, radius * 0.1, center.x, center.y, radius);
    fillGradient.addColorStop(0, "rgba(254, 202, 202, 0.95)");
    fillGradient.addColorStop(1, "rgba(220, 38, 38, 0.92)");

    ctx.save();
    ctx.shadowColor = "rgba(220, 38, 38, 0.28)";
    ctx.shadowBlur = 18;
    ctx.beginPath();
    ctx.arc(center.x, center.y, radius, 0, Math.PI * 2);
    ctx.fillStyle = fillGradient;
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.lineWidth = 2.4;
    ctx.strokeStyle = "rgba(127, 29, 29, 0.96)";
    ctx.stroke();
    ctx.beginPath();
    ctx.setLineDash([8, 6]);
    ctx.arc(center.x, center.y, radius * 1.18, 0, Math.PI * 2);
    ctx.strokeStyle = "rgba(254, 226, 226, 0.9)";
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.restore();
}

function drawRuntimeRectangularObstacle(obstacle, project, scale) {
    const center = project(obstacle.x, obstacle.y);
    const width = obstacle.width * scale;
    const height = obstacle.height * scale;
    const left = center.x - width / 2;
    const top = center.y - height / 2;
    const fill = ctx.createLinearGradient(left, top, left + width, top + height);
    fill.addColorStop(0, "rgba(254, 226, 226, 0.96)");
    fill.addColorStop(1, "rgba(220, 38, 38, 0.88)");

    ctx.save();
    ctx.shadowColor = "rgba(220, 38, 38, 0.26)";
    ctx.shadowBlur = 16;
    roundedRectPath(left, top, width, height, Math.min(14, width * 0.16, height * 0.2));
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.lineWidth = 2.2;
    ctx.strokeStyle = "rgba(127, 29, 29, 0.95)";
    ctx.stroke();
    ctx.restore();
}

function drawCircularObstacle(obstacle, project, scale, isCustom) {
    const center = project(obstacle.x, obstacle.y);
    const radius = obstacle.radius * scale;
    const ringColor = isCustom ? "rgba(217, 119, 6, 0.95)" : "rgba(71, 85, 105, 0.9)";
    const fillGradient = ctx.createRadialGradient(center.x - radius * 0.24, center.y - radius * 0.24, radius * 0.1, center.x, center.y, radius);
    fillGradient.addColorStop(0, isCustom ? "rgba(251, 191, 36, 0.85)" : "rgba(226, 232, 240, 0.98)");
    fillGradient.addColorStop(1, isCustom ? "rgba(180, 83, 9, 0.95)" : "rgba(100, 116, 139, 0.88)");

    ctx.save();
    ctx.shadowColor = isCustom ? "rgba(217, 119, 6, 0.28)" : "rgba(71, 85, 105, 0.24)";
    ctx.shadowBlur = 16;
    ctx.beginPath();
    ctx.arc(center.x, center.y, radius, 0, Math.PI * 2);
    ctx.fillStyle = fillGradient;
    ctx.fill();
    ctx.shadowBlur = 0;

    ctx.lineWidth = 2.4;
    ctx.strokeStyle = ringColor;
    ctx.stroke();

    ctx.beginPath();
    ctx.setLineDash([5, 6]);
    ctx.arc(center.x, center.y, radius * 0.72, 0, Math.PI * 2);
    ctx.strokeStyle = "rgba(255,255,255,0.42)";
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.restore();
}

function drawRectangularObstacle(obstacle, project, scale, isCustom) {
    const center = project(obstacle.x, obstacle.y);
    const width = obstacle.width * scale;
    const height = obstacle.height * scale;
    const left = center.x - width / 2;
    const top = center.y - height / 2;
    const stroke = isCustom ? "rgba(180, 83, 9, 0.96)" : "rgba(71, 85, 105, 0.92)";
    const fill = ctx.createLinearGradient(left, top, left + width, top + height);
    fill.addColorStop(0, isCustom ? "rgba(253, 230, 138, 0.95)" : "rgba(241, 245, 249, 0.96)");
    fill.addColorStop(1, isCustom ? "rgba(217, 119, 6, 0.88)" : "rgba(100, 116, 139, 0.88)");

    ctx.save();
    ctx.shadowColor = isCustom ? "rgba(180, 83, 9, 0.26)" : "rgba(71, 85, 105, 0.22)";
    ctx.shadowBlur = 14;
    roundedRectPath(left, top, width, height, Math.min(14, width * 0.16, height * 0.2));
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.lineWidth = 2.2;
    ctx.strokeStyle = stroke;
    ctx.stroke();

    ctx.save();
    roundedRectPath(left, top, width, height, Math.min(14, width * 0.16, height * 0.2));
    ctx.clip();
    ctx.lineWidth = 2;
    ctx.strokeStyle = "rgba(255,255,255,0.3)";
    for (let offset = -height; offset < width + height; offset += 16) {
        ctx.beginPath();
        ctx.moveTo(left + offset, top + height);
        ctx.lineTo(left + offset + height, top);
        ctx.stroke();
    }
    ctx.restore();
    ctx.restore();
}

function drawGoalMarkers(scenario, project, scale) {
    const start = project(scenario.start.x, scenario.start.y);
    const goal = project(scenario.goal.x, scenario.goal.y);

    ctx.save();
    ctx.fillStyle = "#15803d";
    ctx.beginPath();
    ctx.arc(start.x, start.y, Math.max(6, scenario.robot_radius * scale * 0.95), 0, Math.PI * 2);
    ctx.fill();

    ctx.translate(goal.x, goal.y);
    ctx.fillStyle = "#dc2626";
    ctx.beginPath();
    for (let index = 0; index < 5; index += 1) {
        const outerAngle = ((Math.PI * 2) / 5) * index - Math.PI / 2;
        const innerAngle = outerAngle + Math.PI / 5;
        const outerRadius = 12;
        const innerRadius = 5;
        const outerX = Math.cos(outerAngle) * outerRadius;
        const outerY = Math.sin(outerAngle) * outerRadius;
        const innerX = Math.cos(innerAngle) * innerRadius;
        const innerY = Math.sin(innerAngle) * innerRadius;
        if (index === 0) {
            ctx.moveTo(outerX, outerY);
        } else {
            ctx.lineTo(outerX, outerY);
        }
        ctx.lineTo(innerX, innerY);
    }
    ctx.closePath();
    ctx.fill();
    ctx.restore();
}

function drawTrajectory(result, project) {
    const trajectory = result.trajectory.slice(0, Math.min(state.frame + 1, result.trajectory.length));
    if (trajectory.length < 2) {
        return;
    }

    ctx.save();
    ctx.strokeStyle = colorFor(result.algorithm);
    ctx.lineWidth = 3.2;
    ctx.lineCap = "round";
    ctx.shadowColor = `${colorFor(result.algorithm)}44`;
    ctx.shadowBlur = 10;
    ctx.beginPath();
    trajectory.forEach((point, index) => {
        const projected = project(point.x, point.y);
        if (index === 0) {
            ctx.moveTo(projected.x, projected.y);
        } else {
            ctx.lineTo(projected.x, projected.y);
        }
    });
    ctx.stroke();
    ctx.restore();
}

function drawRobot(result, project, scale, robotRadius) {
    const index = Math.min(state.frame, result.trajectory.length - 1);
    const robot = result.trajectory[index];
    const center = project(robot.x, robot.y);
    const bodyLength = Math.max(24, robotRadius * scale * 2.8);
    const bodyWidth = Math.max(14, robotRadius * scale * 1.7);
    const wheelWidth = bodyLength * 0.18;
    const wheelHeight = bodyWidth * 0.22;

    ctx.save();
    ctx.translate(center.x, center.y);
    ctx.rotate(-robot.theta);

    ctx.fillStyle = "rgba(15, 23, 42, 0.18)";
    roundedRectPath(-bodyLength * 0.42, -bodyWidth * 0.32 + 3, bodyLength * 0.84, bodyWidth * 0.64, bodyWidth * 0.26);
    ctx.fill();

    ctx.fillStyle = colorFor(result.algorithm);
    ctx.strokeStyle = "rgba(15, 23, 42, 0.4)";
    ctx.lineWidth = 1.6;
    roundedRectPath(-bodyLength / 2, -bodyWidth / 2, bodyLength, bodyWidth, bodyWidth * 0.28);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = "rgba(226, 232, 240, 0.82)";
    roundedRectPath(-bodyLength * 0.12, -bodyWidth * 0.32, bodyLength * 0.42, bodyWidth * 0.64, bodyWidth * 0.2);
    ctx.fill();

    ctx.fillStyle = "#0f172a";
    drawWheel(-bodyLength * 0.28, -bodyWidth * 0.52, wheelWidth, wheelHeight);
    drawWheel(bodyLength * 0.12, -bodyWidth * 0.52, wheelWidth, wheelHeight);
    drawWheel(-bodyLength * 0.28, bodyWidth * 0.3, wheelWidth, wheelHeight);
    drawWheel(bodyLength * 0.12, bodyWidth * 0.3, wheelWidth, wheelHeight);

    ctx.fillStyle = "rgba(255, 248, 196, 0.95)";
    ctx.beginPath();
    ctx.arc(bodyLength * 0.42, -bodyWidth * 0.16, 2.6, 0, Math.PI * 2);
    ctx.arc(bodyLength * 0.42, bodyWidth * 0.16, 2.6, 0, Math.PI * 2);
    ctx.fill();

    ctx.restore();
}

function drawWheel(x, y, width, height) {
    roundedRectPath(x, y, width, height, height * 0.4);
    ctx.fill();
}

async function handleCanvasPlacement(event) {
    const scenario = getSelectedScenarioDefinition();
    if (!scenario) {
        return;
    }

    const worldPoint = screenToWorld(event, scenario);
    if (!worldPoint) {
        return;
    }

    try {
        const obstacle = buildObstacleFromInputs(worldPoint.x, worldPoint.y, scenario);
        if (canInsertRuntimeObstacle()) {
            validateRuntimePlacement(obstacle);
            const existingRuntimeEvents = [...getRuntimeObstacleEvents()];
            const activateTime = roundToTwo(getCurrentSimulationTime());
            const runtimeEvent = {
                activate_time: activateTime,
                label: `insertion_${existingRuntimeEvents.length + 1}`,
                obstacle,
            };
            setRuntimeObstacleEvents([...existingRuntimeEvents, runtimeEvent]);
            renderObstacleList();
            const payload = await runCurrentSelection({ startFrame: state.frame });
            if (!payload) {
                setRuntimeObstacleEvents(existingRuntimeEvents);
                refreshPreviewScenario();
                renderObstacleList();
                renderScene();
                return;
            }
            setStatus(`Inserted a live ${obstacle.kind} obstacle at t=${activateTime.toFixed(2)} s. The run was recomputed from the current playback time.`);
            return;
        }

        setCustomObstacles([...getCustomObstacles(), obstacle]);
        markSimulationStale(`Placed a ${obstacle.kind} obstacle at (${obstacle.x.toFixed(2)}, ${obstacle.y.toFixed(2)}). Run the simulation to recompute the path.`);
    } catch (error) {
        setStatus(error.message, true);
    }
}

function screenToWorld(event, scenario) {
    const rect = elements.canvas.getBoundingClientRect();
    const scaleX = elements.canvas.width / rect.width;
    const scaleY = elements.canvas.height / rect.height;
    const canvasX = (event.clientX - rect.left) * scaleX;
    const canvasY = (event.clientY - rect.top) * scaleY;
    const projection = getCanvasProjection(scenario);

    const worldX = (canvasX - projection.padding) / projection.scale;
    const worldY = (projection.height - projection.padding - canvasY) / projection.scale;
    if (worldX < 0 || worldX > scenario.width || worldY < 0 || worldY > scenario.height) {
        return null;
    }

    return { x: worldX, y: worldY };
}

function buildObstacleFromInputs(worldX, worldY, scenario) {
    const kind = elements.obstacleTypeSelect.value;
    if (kind === "circle") {
        const radius = requirePositiveNumber(elements.obstacleRadiusInput.value, "Radius");
        return {
            kind: "circle",
            x: roundToTwo(clampValue(worldX, radius, scenario.width - radius)),
            y: roundToTwo(clampValue(worldY, radius, scenario.height - radius)),
            radius: roundToTwo(radius),
        };
    }

    const width = requirePositiveNumber(elements.obstacleWidthInput.value, "Width");
    const height = requirePositiveNumber(elements.obstacleHeightInput.value, "Height");
    return {
        kind: "rectangle",
        x: roundToTwo(clampValue(worldX, width / 2, scenario.width - width / 2)),
        y: roundToTwo(clampValue(worldY, height / 2, scenario.height - height / 2)),
        width: roundToTwo(width),
        height: roundToTwo(height),
    };
}

function markSimulationStale(message) {
    refreshPreviewScenario();
    resetDisplayedResults();
    renderLegend();
    renderMetrics();
    renderObstacleList();
    renderScene();
    setStatus(message);
}

async function rerunLiveScenario(message) {
    refreshPreviewScenario();
    renderObstacleList();
    const payload = await runCurrentSelection({ startFrame: state.frame });
    if (payload) {
        setStatus(message);
    }
}

function resetDisplayedResults() {
    state.results = [];
    state.frame = 0;
    state.isPlaying = false;
    state.lastTick = 0;
    elements.playButton.textContent = "Play";
    syncFrameSlider();
}

function refreshPreviewScenario() {
    const baseScenario = getSelectedScenarioDefinition();
    if (!baseScenario) {
        return;
    }
    state.scenario = composeScenario(baseScenario, getCustomObstacles(), getRuntimeObstacleEvents());
}

function getSelectedScenarioDefinition() {
    return state.meta?.scenarios.find((scenario) => scenario.name === elements.scenarioSelect.value) || null;
}

function ensureScenarioStores() {
    const scenarioName = elements.scenarioSelect.value;
    if (!state.customObstaclesByScenario[scenarioName]) {
        state.customObstaclesByScenario[scenarioName] = [];
    }
    if (!state.runtimeObstacleEventsByScenario[scenarioName]) {
        state.runtimeObstacleEventsByScenario[scenarioName] = [];
    }
}

function getCustomObstacles() {
    ensureScenarioStores();
    return state.customObstaclesByScenario[elements.scenarioSelect.value];
}

function setCustomObstacles(obstacles) {
    state.customObstaclesByScenario[elements.scenarioSelect.value] = obstacles;
}

function getRuntimeObstacleEvents() {
    ensureScenarioStores();
    return state.runtimeObstacleEventsByScenario[elements.scenarioSelect.value];
}

function setRuntimeObstacleEvents(events) {
    state.runtimeObstacleEventsByScenario[elements.scenarioSelect.value] = events;
}

function composeScenario(baseScenario, customObstacles, runtimeObstacleEvents) {
    return {
        ...baseScenario,
        obstacles: [...baseScenario.obstacles, ...customObstacles],
        dynamic_obstacles: [...(baseScenario.dynamic_obstacles || [])],
        runtime_obstacle_events: [...(runtimeObstacleEvents || [])],
    };
}

function getCurrentRuntimeObstacleSnapshots(scenario) {
    const currentTime = getCurrentSimulationTime();
    return (scenario.runtime_obstacle_events || [])
        .filter((event) => event.activate_time <= currentTime + 1e-9)
        .map((event) => ({
            ...event.obstacle,
            label: event.label,
            activate_time: event.activate_time,
        }));
}

function getCurrentDynamicObstacleSnapshots(scenario) {
    if (state.results.length > 0) {
        const histories = state.results.map((result) => result.dynamic_obstacle_history || []);
        const longestHistory = histories.sort((left, right) => right.length - left.length)[0] || [];
        if (longestHistory.length > 0) {
            return longestHistory[Math.min(state.frame, longestHistory.length - 1)] || [];
        }
    }

    return (scenario.dynamic_obstacles || []).map((obstacle) => snapshotDynamicObstacle(obstacle, scenario, state.previewTimeSeconds));
}

function canInsertRuntimeObstacle() {
    return state.results.length > 0;
}

function getCurrentSimulationTime() {
    if (state.results.length > 0) {
        return state.frame * Number(elements.dtInput.value);
    }
    return state.previewTimeSeconds;
}

function getCurrentRobotStates() {
    return state.results
        .map((result) => result.trajectory[Math.min(state.frame, result.trajectory.length - 1)])
        .filter(Boolean);
}

function validateRuntimePlacement(obstacle) {
    const scenario = state.scenario || getSelectedScenarioDefinition();
    if (!scenario) {
        throw new Error("No active scenario is loaded.");
    }

    if (!obstacleFitsInBounds(obstacle, scenario)) {
        throw new Error("The obstacle must stay within the scenario bounds.");
    }

    const robotBuffer = scenario.robot_radius + scenario.safety_margin * 0.25;
    for (const robotState of getCurrentRobotStates()) {
        if (distanceFromPointToObstacle(robotState.x, robotState.y, obstacle) <= robotBuffer) {
            throw new Error("Live obstacle placement is blocked because it overlaps the vehicle at the current time.");
        }
    }

    const activeObstacles = [
        ...scenario.obstacles,
        ...getCurrentRuntimeObstacleSnapshots(scenario),
        ...getCurrentDynamicObstacleSnapshots(scenario),
    ];
    const obstacleBuffer = scenario.safety_margin * 0.15;
    for (const activeObstacle of activeObstacles) {
        if (distanceBetweenObstacles(obstacle, activeObstacle) <= obstacleBuffer) {
            throw new Error("Live obstacle placement is blocked because it overlaps an active obstacle.");
        }
    }
}

function snapshotDynamicObstacle(obstacle, scenario, timeValue) {
    const minX = obstacle.min_x ?? obstacle.radius;
    const maxX = obstacle.max_x ?? scenario.width - obstacle.radius;
    const minY = obstacle.min_y ?? obstacle.radius;
    const maxY = obstacle.max_y ?? scenario.height - obstacle.radius;

    return {
        x: reflectedCoordinate(obstacle.x, obstacle.vx || 0, minX, maxX, timeValue),
        y: reflectedCoordinate(obstacle.y, obstacle.vy || 0, minY, maxY, timeValue),
        radius: obstacle.radius,
        label: obstacle.label,
    };
}

function reflectedCoordinate(origin, velocity, lower, upper, timeValue) {
    if (upper <= lower || Math.abs(velocity) <= 1e-9) {
        return clampValue(origin, lower, upper);
    }

    const span = upper - lower;
    const period = span * 2;
    const shifted = (origin - lower) + velocity * timeValue;
    const wrapped = ((shifted % period) + period) % period;
    if (wrapped <= span) {
        return lower + wrapped;
    }
    return upper - (wrapped - span);
}

function obstacleFitsInBounds(obstacle, scenario) {
    if (obstacle.kind === "circle") {
        return (
            obstacle.radius <= obstacle.x
            && obstacle.x <= scenario.width - obstacle.radius
            && obstacle.radius <= obstacle.y
            && obstacle.y <= scenario.height - obstacle.radius
        );
    }

    return (
        obstacle.width / 2 <= obstacle.x
        && obstacle.x <= scenario.width - obstacle.width / 2
        && obstacle.height / 2 <= obstacle.y
        && obstacle.y <= scenario.height - obstacle.height / 2
    );
}

function distanceFromPointToObstacle(x, y, obstacle) {
    if (obstacle.kind === "circle" || obstacle.kind === "dynamic_circle") {
        return Math.hypot(x - obstacle.x, y - obstacle.y) - obstacle.radius;
    }

    const dx = Math.max(Math.abs(x - obstacle.x) - obstacle.width / 2, 0);
    const dy = Math.max(Math.abs(y - obstacle.y) - obstacle.height / 2, 0);
    const insideDx = Math.abs(x - obstacle.x) - obstacle.width / 2;
    const insideDy = Math.abs(y - obstacle.y) - obstacle.height / 2;
    const outsideDistance = Math.hypot(dx, dy);
    const insideDistance = Math.min(Math.max(insideDx, insideDy), 0);
    return outsideDistance + insideDistance;
}

function distanceBetweenObstacles(first, second) {
    const firstKind = first.kind === "dynamic_circle" ? "circle" : first.kind;
    const secondKind = second.kind === "dynamic_circle" ? "circle" : second.kind;

    if (firstKind === "circle" && secondKind === "circle") {
        return Math.hypot(first.x - second.x, first.y - second.y) - first.radius - second.radius;
    }

    if (firstKind === "rectangle" && secondKind === "rectangle") {
        const dx = Math.abs(first.x - second.x) - (first.width + second.width) / 2;
        const dy = Math.abs(first.y - second.y) - (first.height + second.height) / 2;
        const outsideDistance = Math.hypot(Math.max(dx, 0), Math.max(dy, 0));
        const insideDistance = Math.min(Math.max(dx, dy), 0);
        return outsideDistance + insideDistance;
    }

    const circle = firstKind === "circle" ? first : second;
    const rectangle = firstKind === "rectangle" ? first : second;
    const nearestX = clampValue(circle.x, rectangle.x - rectangle.width / 2, rectangle.x + rectangle.width / 2);
    const nearestY = clampValue(circle.y, rectangle.y - rectangle.height / 2, rectangle.y + rectangle.height / 2);
    return Math.hypot(circle.x - nearestX, circle.y - nearestY) - circle.radius;
}

function roundedRectPath(x, y, width, height, radius) {
    const corner = Math.min(radius, width / 2, height / 2);
    ctx.beginPath();
    ctx.moveTo(x + corner, y);
    ctx.lineTo(x + width - corner, y);
    ctx.quadraticCurveTo(x + width, y, x + width, y + corner);
    ctx.lineTo(x + width, y + height - corner);
    ctx.quadraticCurveTo(x + width, y + height, x + width - corner, y + height);
    ctx.lineTo(x + corner, y + height);
    ctx.quadraticCurveTo(x, y + height, x, y + height - corner);
    ctx.lineTo(x, y + corner);
    ctx.quadraticCurveTo(x, y, x + corner, y);
    ctx.closePath();
}

function updateFrameReadout() {
    const maxFrame = Number(elements.frameSlider.max);
    elements.frameReadout.textContent = `Frame ${state.frame} / ${maxFrame}`;
}

function setStatus(message, isError = false) {
    elements.statusBanner.textContent = message;
    elements.statusBanner.style.background = isError ? "rgba(220, 38, 38, 0.08)" : "rgba(15, 118, 110, 0.08)";
    elements.statusBanner.style.borderColor = isError ? "rgba(220, 38, 38, 0.14)" : "rgba(15, 118, 110, 0.12)";
    elements.statusBanner.style.color = isError ? "#991b1b" : "#0f5132";
}

function requirePositiveNumber(rawValue, label) {
    const parsed = Number(rawValue);
    if (!Number.isFinite(parsed) || parsed <= 0) {
        throw new Error(`${label} must be a positive number.`);
    }
    return parsed;
}

function roundToTwo(value) {
    return Math.round(value * 100) / 100;
}

function clampValue(value, min, max) {
    return Math.max(min, Math.min(value, max));
}

function colorFor(algorithm) {
    return COLORS[algorithm] || "#334155";
}

function buildMetricBackground(algorithm) {
    if (algorithm === "astar") {
        return "linear-gradient(135deg, #0f766e, #115e59)";
    }
    if (algorithm === "apf") {
        return "linear-gradient(135deg, #d97706, #b45309)";
    }
    if (algorithm === "dwa") {
        return "linear-gradient(135deg, #1d4ed8, #1e40af)";
    }
    return "linear-gradient(135deg, #475569, #334155)";
}

bootstrap();
