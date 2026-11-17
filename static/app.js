const fallbackOptions = {
  task_modes: [
    { id: "restock", name_zh: "上架" },
    { id: "pick", name_zh: "拣货" },
    { id: "full_flow", name_zh: "全流程" },
  ],
  drinks: [
    ["coca_cola", "可口可乐", "Coca-Cola", "Coca-Cola", "#d92d20"],
    ["ad_calcium_milk", "AD钙奶", "AD Calcium Milk", "AD Calcium Milk", "#f2b705"],
    ["daily_c_orange", "味全每日C橙汁", "Daily C Orange Juice", "Daily C Orange Juice", "#f97316"],
    ["daily_c_grape", "味全每日C葡萄汁", "Daily C Grape Juice", "Daily C Grape Juice", "#7c3aed"],
    ["wanglaoji", "王老吉", "wanglaoji", "wanglaoji", "#b42318"],
    ["dahongpao_milk_tea", "果子熟了大红袍乌龙轻乳茶", "Dahongpao Milk Tea", "Dahongpao Milk Tea", "#9a3412"],
    ["sprite", "雪碧", "Sprite", "Sprite", "#16a34a"],
    ["hk_orange_fanta", "港版芬达橙子味", "HK Orange Fanta", "HK Orange Fanta", "#fb923c"],
    ["aojiru", "伊藤园青汁", "Aojiru", "Aojiru", "#15803d"],
  ].map(([id, name_zh, brand_zh, prompt, tone]) => ({ id, name_zh, brand_zh, prompt, tone })),
  arms: [
    { id: "right", name_zh: "右臂", camera_frame: "cam_right_color_optical_frame" },
    { id: "left", name_zh: "左臂", camera_frame: "cam_left_color_optical_frame" },
  ],
  shelf: {
    layers: ["1", "2", "3"].map((id) => ({ id, name_zh: `第${["一", "二", "三"][Number(id) - 1]}层` })),
    positions: Array.from({ length: 12 }, (_, index) => String(index + 1)).map((id) => ({ id, name_zh: `位置 ${id}` })),
  },
  defaults: { scene_id: 1, shelf_profile_id: "shelf_1", gripper_width_m: 0.08 },
  shelf_profiles: [
    {
      id: "shelf_1",
      name_zh: "货架1（原货架）",
      scene_id: 1,
      default_layer: "2",
      default_position: "3",
      configured_slots: [],
      configured_pick_arms: [],
      slot_owners: {},
      slot_routes_required: false,
      slot_routes: {},
      full_flow: { configured_left_slots: [], right_destination_configured: false, routes_ready: true },
    },
    {
      id: "shelf_2",
      name_zh: "货架2（新大货架）",
      scene_id: 3,
      default_layer: "1",
      default_position: "5",
      configured_slots: [],
      configured_pick_arms: [],
      slot_owners: {
        "1": "left", "2": "left", "3": "left",
        "4": "right", "5": "right", "6": "right",
        "7": "left", "8": "left", "9": "left",
        "10": "right", "11": "right", "12": "right",
      },
      slot_routes_required: true,
      slot_routes: {
        standalone_restock: {
          "right:1:5": { config: "right_route_to_place_scene_003_layer_1_position_5.yaml", enabled: false },
        },
        standalone_return: {
          "right:1:5": { config: "right_route_to_photo_scene_003_layer_1_position_5.yaml", enabled: false },
        },
      },
      full_flow: { configured_left_slots: [], right_destination_configured: false, routes_ready: false },
    },
  ],
  place_pose_config: { configured_slots: [] },
  pick_place_pose_config: { configured_arms: [] },
  full_flow: {
    configured_left_slots: [],
    right_destination_configured: false,
    minimum_retreat_m: 0.08,
  },
  recording: { enabled: true, target_dir: "", storage: "mcap", topics: [] },
};

const state = {
  options: fallbackOptions,
  selectedTaskMode: "restock",
  selectedDrinkId: "dahongpao_milk_tea",
  selectedLayer: "1",
  selectedPosition: "1",
  selectedShelfProfileId: "shelf_1",
  fullFlowPreview: null,
  fullFlowPreviewing: false,
  busy: false,
  combinedStarting: false,
  lastRecording: null,
  polling: null,
};

const drinkGrid = document.getElementById("drinkGrid");
const shelfGrid = document.getElementById("shelfGrid");
const taskModeSwitch = document.getElementById("taskModeSwitch");
const promptPreview = document.getElementById("promptPreview");
const slotStatus = document.getElementById("slotStatus");
const runSummary = document.getElementById("runSummary");
const statusOutput = document.getElementById("statusOutput");
const executeBtn = document.getElementById("executeBtn");
const executeRecordBtn = document.getElementById("executeRecordBtn");
const stopExecuteBtn = document.getElementById("stopExecuteBtn");
const refreshBtn = document.getElementById("refreshBtn");
const recordStartBtn = document.getElementById("recordStartBtn");
const recordStopBtn = document.getElementById("recordStopBtn");
const recordSaveBtn = document.getElementById("recordSaveBtn");
const recordDiscardBtn = document.getElementById("recordDiscardBtn");
const recordStatusText = document.getElementById("recordStatusText");
const recordDetail = document.getElementById("recordDetail");
const rosStatus = document.getElementById("rosStatus");
const armSelect = document.getElementById("armSelect");
const shelfProfileSelect = document.getElementById("shelfProfileSelect");
const gripperInput = document.getElementById("gripperInput");
const placeSectionTitle = document.getElementById("placeSectionTitle");
const armField = document.getElementById("armField");
const fullFlowPanel = document.getElementById("fullFlowPanel");
const fullFlowPreviewBtn = document.getElementById("fullFlowPreviewBtn");
const fullFlowLeftStep = document.getElementById("fullFlowLeftStep");
const fullFlowRightStep = document.getElementById("fullFlowRightStep");
const fullFlowReadiness = document.getElementById("fullFlowReadiness");

init();

async function init() {
  renderOptions();
  bindEvents();
  try {
    const options = await apiGet("/api/options");
    state.options = { ...fallbackOptions, ...options };
    state.selectedShelfProfileId = String(
      options.defaults?.shelf_profile_id || "shelf_1"
    );
    const defaultProfile = selectedShelfProfile();
    state.selectedLayer = String(defaultProfile?.default_layer || "1");
    state.selectedPosition = String(defaultProfile?.default_position || "1");
    gripperInput.value = String(options.defaults?.gripper_width_m ?? 0.08);
    renderOptions();
  } catch (error) {
    setRunSummary(`选项读取失败，使用本地默认：${error.message}`, true);
  }
  await refreshStatus();
}

function bindEvents() {
  taskModeSwitch.addEventListener("click", (event) => {
    const button = event.target.closest("[data-task-mode]");
    if (!button) return;
    state.selectedTaskMode = button.dataset.taskMode;
    ensureSelectedSlotForMode();
    renderTaskModeSwitch();
    updatePromptPreview();
    updateSlotStatus();
    if (state.selectedTaskMode === "full_flow") {
      void refreshFullFlowPreview();
    }
  });
  drinkGrid.addEventListener("click", (event) => {
    const button = event.target.closest("[data-drink-id]");
    if (!button) return;
    state.selectedDrinkId = button.dataset.drinkId;
    state.fullFlowPreview = null;
    renderDrinkGrid();
    updatePromptPreview();
    if (state.selectedTaskMode === "full_flow") {
      void refreshFullFlowPreview();
    }
  });
  shelfGrid.addEventListener("click", (event) => {
    const button = event.target.closest("[data-slot]");
    if (!button) return;
    state.selectedLayer = button.dataset.layer;
    state.selectedPosition = button.dataset.position;
    const owner = shelfSlotOwner(state.selectedPosition);
    if (owner && state.selectedTaskMode === "restock") {
      armSelect.value = owner;
    }
    state.fullFlowPreview = null;
    renderShelfGrid();
    updateSlotStatus();
    if (state.selectedTaskMode === "full_flow") {
      void refreshFullFlowPreview();
    }
  });
  armSelect.addEventListener("change", () => {
    ensureSelectedSlotForMode();
    updateSlotStatus();
  });
  shelfProfileSelect.addEventListener("change", () => {
    state.selectedShelfProfileId = shelfProfileSelect.value;
    const profile = selectedShelfProfile();
    state.selectedLayer = String(profile?.default_layer || "1");
    state.selectedPosition = String(profile?.default_position || "1");
    ensureSelectedSlotForMode();
    state.fullFlowPreview = null;
    renderOptions();
    if (state.selectedTaskMode === "full_flow") {
      void refreshFullFlowPreview();
    }
  });
  gripperInput.addEventListener("change", invalidateFullFlowPreview);
  executeBtn.addEventListener("click", executeTask);
  executeRecordBtn.addEventListener("click", executeAndRecordTask);
  fullFlowPreviewBtn.addEventListener("click", refreshFullFlowPreview);
  stopExecuteBtn.addEventListener("click", stopExecution);
  refreshBtn.addEventListener("click", refreshStatus);
  recordStartBtn.addEventListener("click", startRecording);
  recordStopBtn.addEventListener("click", () => recordingAction("stop"));
  recordSaveBtn.addEventListener("click", () => recordingAction("save"));
  recordDiscardBtn.addEventListener("click", () => recordingAction("discard"));
}

function renderOptions() {
  renderTaskModeSwitch();
  renderShelfProfileSelect();
  renderArmSelect();
  renderDrinkGrid();
  renderShelfGrid();
  updatePromptPreview();
  updateSlotStatus();
}

function renderShelfProfileSelect() {
  const profiles = state.options.shelf_profiles || fallbackOptions.shelf_profiles;
  shelfProfileSelect.innerHTML = profiles.map((profile) => `
    <option value="${escapeHtml(profile.id)}">
      ${escapeHtml(profile.name_zh)} · Scene ${escapeHtml(profile.scene_id)}
    </option>
  `).join("");
  if (!profiles.some((profile) => profile.id === state.selectedShelfProfileId)) {
    state.selectedShelfProfileId = profiles[0]?.id || "shelf_1";
  }
  shelfProfileSelect.value = state.selectedShelfProfileId;
}

function renderTaskModeSwitch() {
  const modes = state.options.task_modes || fallbackOptions.task_modes;
  taskModeSwitch.innerHTML = modes.map((mode) => {
    const active = mode.id === state.selectedTaskMode;
    return `
      <button class="${active ? "active" : ""}" type="button" data-task-mode="${escapeHtml(mode.id)}">
        ${escapeHtml(mode.name_zh)}
      </button>
    `;
  }).join("");
}

function renderArmSelect() {
  const current = armSelect.value || "right";
  armSelect.innerHTML = state.options.arms
    .map((arm) => `<option value="${escapeHtml(arm.id)}">${escapeHtml(arm.name_zh)}</option>`)
    .join("");
  armSelect.value = state.options.arms.some((arm) => arm.id === current) ? current : "right";
}

function renderDrinkGrid() {
  drinkGrid.innerHTML = state.options.drinks.map((drink) => {
    const active = drink.id === state.selectedDrinkId;
    return `
      <button class="drink-card ${active ? "active" : ""}" type="button" data-drink-id="${escapeHtml(drink.id)}">
        <span class="drink-swatch" style="background:${escapeHtml(drink.tone || "#667085")}"></span>
        <strong>${escapeHtml(drink.name_zh)}</strong>
        <span>${escapeHtml(drink.brand_zh)}</span>
      </button>
    `;
  }).join("");
}

function renderShelfGrid() {
  const layers = state.options.shelf?.layers || fallbackOptions.shelf.layers;
  const positions = state.options.shelf?.positions || fallbackOptions.shelf.positions;
  const configuredArm = state.selectedTaskMode === "full_flow" ? "left" : armSelect.value;
  shelfGrid.innerHTML = layers.map((layer) => `
    <div class="shelf-row">
      <div class="shelf-layer">${escapeHtml(layer.name_zh)}</div>
      <div class="slot-row">
        ${positions.map((position) => {
          const owner = shelfSlotOwner(position.id);
          const unavailableForFullFlow = state.selectedTaskMode === "full_flow" && owner === "right";
          const active = layer.id === state.selectedLayer && position.id === state.selectedPosition;
          const slotArm = owner || configuredArm;
          const configured = state.selectedTaskMode === "full_flow"
            ? fullFlowLeftSlotConfigured(layer.id, position.id)
            : slotConfigured(slotArm, layer.id, position.id);
          return `
            <button
              class="slot-button ${active ? "active" : ""} ${configured ? "configured" : ""}"
              type="button"
              data-slot
              data-layer="${escapeHtml(layer.id)}"
              data-position="${escapeHtml(position.id)}"
              title="${escapeHtml(owner ? `位置${position.id}固定使用${armLabel(owner)}` : `位置${position.id}`)}"
              aria-label="${escapeHtml(owner ? `位置${position.id}，${armLabel(owner)}` : `位置${position.id}`)}"
              ${unavailableForFullFlow ? "disabled" : ""}
            >
              ${escapeHtml(position.id)}${owner ? `<small>${owner === "left" ? "左" : "右"}</small>` : ""}
            </button>
          `;
        }).join("")}
      </div>
    </div>
  `).join("");
}

function updatePromptPreview() {
  const drink = selectedDrink();
  promptPreview.textContent = state.selectedTaskMode === "full_flow"
    ? `左右腕相机视觉提示词: ${drink?.prompt || "-"}`
    : `${taskModeLabel(state.selectedTaskMode)}视觉提示词: ${drink?.prompt || "-"}`;
}

function updateSlotStatus() {
  renderShelfGrid();
  const fullFlow = state.selectedTaskMode === "full_flow";
  armField.classList.toggle("hidden", fullFlow);
  fullFlowPanel.classList.toggle("hidden", !fullFlow);
  if (fullFlow) {
    const leftConfigured = fullFlowLeftSlotConfigured(state.selectedLayer, state.selectedPosition);
    const rightConfigured = Boolean(selectedShelfProfile()?.full_flow?.right_destination_configured);
    placeSectionTitle.textContent = "左臂上架位置";
    shelfGrid.classList.remove("hidden");
    slotStatus.textContent = leftConfigured
      ? `${shelfProfileLabel()} · 左臂第${state.selectedLayer}层 位置${state.selectedPosition}已配置；右臂最终点${rightConfigured ? "已配置" : "未配置"}`
      : `${shelfProfileLabel()} · 左臂第${state.selectedLayer}层 位置${state.selectedPosition}未配置，禁止启动全流程`;
    executeBtn.textContent = "执行全流程";
    executeRecordBtn.textContent = "全流程并录制";
    renderFullFlowReadiness();
    return;
  }
  if (state.selectedTaskMode === "pick") {
    const configured = pickPlaceConfigured(armSelect.value);
    const returnToStart = armSelect.value === "right";
    placeSectionTitle.textContent = returnToStart ? "拣货返回" : "拣货放置";
    shelfGrid.classList.add("hidden");
    slotStatus.textContent = returnToStart
      ? `${shelfProfileLabel()} · 回固定初始位置识别 → 抓取 → 撤离货架 → 夹紧返回固定初始位置`
      : configured
      ? `${shelfProfileLabel()} · ${armLabel(armSelect.value)} 拣货放置位姿已配置`
      : `${shelfProfileLabel()} · ${armLabel(armSelect.value)} 拣货放置位姿未配置，使用状态机 pick_place 默认位姿`;
    executeBtn.textContent = "执行拣货";
    executeRecordBtn.textContent = "拣货并录制";
    syncExecuteAvailability();
    return;
  }
  placeSectionTitle.textContent = "货架位置";
  shelfGrid.classList.remove("hidden");
  const configured = slotConfigured(armSelect.value, state.selectedLayer, state.selectedPosition);
  const routeReady = slotRouteReady("standalone_restock", armSelect.value, state.selectedLayer, state.selectedPosition);
  const returnRouteReady = slotRouteReady("standalone_return", armSelect.value, state.selectedLayer, state.selectedPosition);
  const routeRequired = Boolean(selectedShelfProfile()?.slot_routes_required) || armSelect.value === "right";
  const ownerText = shelfSlotOwner(state.selectedPosition)
    ? `（固定${armLabel(shelfSlotOwner(state.selectedPosition))}）`
    : "";
  slotStatus.textContent = !configured
    ? `${shelfProfileLabel()} · 第${state.selectedLayer}层位置${state.selectedPosition}${ownerText}未配置放置位姿`
    : (routeRequired && !routeReady
      ? `${shelfProfileLabel()} · 第${state.selectedLayer}层位置${state.selectedPosition}${ownerText}位姿已保存，专用途经点尚未示教，禁止执行`
      : (Boolean(selectedShelfProfile()?.slot_routes_required) && !returnRouteReady
        ? `${shelfProfileLabel()} · 第${state.selectedLayer}层位置${state.selectedPosition}${ownerText}放置路线已完成，返回途经点尚未示教，禁止执行`
        : `${shelfProfileLabel()} · ${armLabel(armSelect.value)} 第${state.selectedLayer}层位置${state.selectedPosition}${ownerText}已配置，可执行`));
  executeBtn.textContent = "执行上架";
  executeRecordBtn.textContent = "上架并录制";
  syncExecuteAvailability();
}

function selectedDrink() {
  return state.options.drinks.find((drink) => drink.id === state.selectedDrinkId) || state.options.drinks[0];
}

function selectedShelfProfile() {
  const profiles = state.options.shelf_profiles || fallbackOptions.shelf_profiles;
  return profiles.find((profile) => profile.id === state.selectedShelfProfileId)
    || profiles[0];
}

function shelfProfileLabel() {
  return selectedShelfProfile()?.name_zh || state.selectedShelfProfileId;
}

function shelfSlotOwner(position) {
  const owner = selectedShelfProfile()?.slot_owners?.[String(position)];
  return owner === "left" || owner === "right" ? owner : "";
}

function ensureSelectedSlotForMode() {
  const owner = shelfSlotOwner(state.selectedPosition);
  if (state.selectedTaskMode === "full_flow" && owner === "right") {
    state.selectedPosition = "1";
    return;
  }
  if (state.selectedTaskMode === "restock" && owner && owner !== armSelect.value) {
    const positions = state.options.shelf?.positions || fallbackOptions.shelf.positions;
    const compatible = positions.find((position) => shelfSlotOwner(position.id) === armSelect.value);
    if (compatible) state.selectedPosition = compatible.id;
  }
}

function slotConfigured(arm, layer, position) {
  const keys = selectedShelfProfile()?.configured_slots || [];
  return keys.includes(`${arm}:${layer}:${position}`);
}

function pickPlaceConfigured(arm) {
  const arms = selectedShelfProfile()?.configured_pick_arms || [];
  return arms.includes(arm);
}

function slotRouteReady(routeKey, arm, layer, position) {
  const profile = selectedShelfProfile();
  const status = profile?.slot_routes?.[routeKey]?.[`${arm}:${layer}:${position}`];
  if (profile?.slot_routes_required) return Boolean(status?.enabled);
  if (routeKey === "standalone_restock") {
    return arm !== "right" || Boolean(profile?.routes?.standalone_right_restock?.enabled);
  }
  return Boolean(profile?.routes?.[routeKey]?.enabled);
}

function fullFlowLeftSlotConfigured(layer, position) {
  const keys = selectedShelfProfile()?.full_flow?.configured_left_slots || [];
  return keys.includes(`left:${layer}:${position}`);
}

function armLabel(armId) {
  return state.options.arms.find((arm) => arm.id === armId)?.name_zh || armId;
}

function taskModeLabel(modeId) {
  return (state.options.task_modes || fallbackOptions.task_modes)
    .find((mode) => mode.id === modeId)?.name_zh || modeId;
}

function placeSummary(job) {
  if ((job.task_mode || "restock") === "full_flow") {
    return `左臂第${job.left_shelf_layer || job.shelf_layer || "-"}层位置${job.left_shelf_position || job.shelf_position || "-"} → 右臂最终点`;
  }
  if ((job.task_mode || "restock") === "pick") {
    return job.arm === "right" ? "夹紧返回固定初始位置" : "拣货放置点";
  }
  return `第${job.shelf_layer || "-"}层位置${job.shelf_position || "-"}`;
}

function fullFlowRequestBody(autoSaveRecording = false) {
  const profile = selectedShelfProfile();
  return {
    drink_id: selectedDrink()?.id || "",
    left_shelf_layer: state.selectedLayer,
    left_shelf_position: state.selectedPosition,
    right_destination: "default",
    shelf_profile_id: profile?.id || "shelf_1",
    scene_id: Number(profile?.scene_id || 1),
    gripper_width_m: Number(gripperInput.value || 0.08),
    auto_save_recording: autoSaveRecording,
  };
}

function invalidateFullFlowPreview() {
  state.fullFlowPreview = null;
  if (state.selectedTaskMode === "full_flow") {
    renderFullFlowReadiness();
  }
}

async function refreshFullFlowPreview() {
  if (state.fullFlowPreviewing) return state.fullFlowPreview;
  state.fullFlowPreviewing = true;
  renderFullFlowReadiness();
  try {
    state.fullFlowPreview = await apiPost("/api/full-flow/preview", fullFlowRequestBody());
  } catch (error) {
    state.fullFlowPreview = {
      ready: false,
      issues: [{ message: error.message }],
    };
  } finally {
    state.fullFlowPreviewing = false;
    renderFullFlowReadiness();
  }
  return state.fullFlowPreview;
}

function renderFullFlowReadiness() {
  if (state.selectedTaskMode !== "full_flow") return;
  const leftConfigured = fullFlowLeftSlotConfigured(state.selectedLayer, state.selectedPosition);
  const rightConfigured = Boolean(selectedShelfProfile()?.full_flow?.right_destination_configured);
  const routesReady = slotRouteReady("full_flow_left_restock", "left", state.selectedLayer, state.selectedPosition)
    && slotRouteReady("full_flow_right_pick", "right", state.selectedLayer, state.selectedPosition);
  fullFlowLeftStep.className = leftConfigured ? "configured" : "blocked";
  fullFlowRightStep.className = rightConfigured ? "configured" : "blocked";
  fullFlowPreviewBtn.disabled = state.fullFlowPreviewing || state.busy;
  if (state.fullFlowPreviewing) {
    fullFlowReadiness.className = "full-flow-readiness checking";
    fullFlowReadiness.textContent = "正在检查位姿、ROS 服务和状态机安全参数…";
  } else if (state.fullFlowPreview?.ready) {
    const parameters = state.fullFlowPreview.state_machine?.parameters || {};
    fullFlowReadiness.className = "full-flow-readiness ready";
    fullFlowReadiness.textContent = `预检通过 · 预抓取 ${formatMeters(parameters.pre_grasp_offset_m)} · 放置后至少回退 ${formatMeters(parameters.retreat_after_place_offset_m)}`;
  } else {
    const issues = state.fullFlowPreview?.issues || [];
    const fallback = !routesReady
      ? `${shelfProfileLabel()}的全流程途经点尚未示教启用`
      : (!leftConfigured
      ? `左臂第${state.selectedLayer}层位置${state.selectedPosition}未配置`
      : (!rightConfigured ? "右臂最终放置位未配置" : "请先检查全流程"));
    fullFlowReadiness.className = "full-flow-readiness blocked";
    fullFlowReadiness.textContent = issues.length
      ? issues.map((issue) => issue.message || issue.code).join("；")
      : fallback;
  }
  syncExecuteAvailability();
}

function syncExecuteAvailability() {
  const fullFlowBlocked = state.selectedTaskMode === "full_flow"
    && !Boolean(state.fullFlowPreview?.ready);
  const recording = state.lastRecording || {};
  const recordingBlocked = recording.status === "starting"
    || (recording.status === "recording" && !recording.session?.ready_at);
  const combinedBlocked = recording.enabled === false || recording.status !== "idle";
  const profile = selectedShelfProfile();
  const restockOwner = shelfSlotOwner(state.selectedPosition);
  const restockBlocked = state.selectedTaskMode === "restock" && (
    (restockOwner && restockOwner !== armSelect.value)
    || !slotConfigured(armSelect.value, state.selectedLayer, state.selectedPosition)
    || ((Boolean(profile?.slot_routes_required) || armSelect.value === "right")
      && !slotRouteReady("standalone_restock", armSelect.value, state.selectedLayer, state.selectedPosition))
    || (Boolean(profile?.slot_routes_required)
      && !slotRouteReady("standalone_return", armSelect.value, state.selectedLayer, state.selectedPosition))
  );
  executeBtn.disabled = state.busy || state.combinedStarting || fullFlowBlocked || restockBlocked || recordingBlocked;
  executeRecordBtn.disabled = state.busy
    || state.combinedStarting
    || fullFlowBlocked
    || restockBlocked
    || combinedBlocked;
}

function formatMeters(value) {
  const number = Number(value);
  return Number.isFinite(number) ? `${Math.round(number * 100)} cm` : "-";
}

function executeRequestBody(autoSaveRecording = false) {
  const drink = selectedDrink();
  const profile = selectedShelfProfile();
  return {
    task_mode: state.selectedTaskMode,
    drink_id: drink?.id || "",
    arm: armSelect.value,
    shelf_layer: state.selectedLayer,
    shelf_position: state.selectedPosition,
    shelf_profile_id: profile?.id || "shelf_1",
    scene_id: Number(profile?.scene_id || 1),
    gripper_width_m: Number(gripperInput.value || 0.08),
    auto_save_recording: autoSaveRecording,
  };
}

async function executeTask() {
  const drink = selectedDrink();
  if (!drink) {
    setRunSummary("请选择饮料", true);
    return;
  }
  if (state.selectedTaskMode === "full_flow") {
    await executeFullFlow(drink);
    return;
  }
  executeBtn.disabled = true;
  const placeText = state.selectedTaskMode === "pick"
    ? (armSelect.value === "right" ? "夹紧返回固定初始位置" : "拣货放置点")
    : `第${state.selectedLayer}层位置${state.selectedPosition}`;
  setRunSummary(`已提交${taskModeLabel(state.selectedTaskMode)}：${drink.name_zh} / ${shelfProfileLabel()} / ${armLabel(armSelect.value)} / ${placeText}`, false);
  try {
    const payload = await apiPost("/api/execute", executeRequestBody());
    renderStatus(payload);
    startPolling();
  } catch (error) {
    setRunSummary(`启动失败：${error.message}`, true);
    if (error.payload) {
      renderStatus(error.payload);
    }
  } finally {
    await refreshStatus();
  }
}

async function executeFullFlow(drink) {
  executeBtn.disabled = true;
  const preview = await refreshFullFlowPreview();
  if (!preview?.ready) {
    const message = (preview?.issues || [])
      .map((issue) => issue.message || issue.code)
      .join("；") || "全流程预检未通过";
    setRunSummary(`无法启动全流程：${message}`, true);
    syncExecuteAvailability();
    return;
  }
  const confirmed = window.confirm(
    `确认执行全流程？\n\n货架：${shelfProfileLabel()} · Scene ${selectedShelfProfile()?.scene_id}\n饮料：${drink.name_zh}\n左臂：第${state.selectedLayer}层位置${state.selectedPosition}\n右臂：最终放置点 default\n\n左臂完整返回拍照位后，右臂才会启动。`
  );
  if (!confirmed) {
    setRunSummary("已取消启动全流程", false);
    syncExecuteAvailability();
    return;
  }
  setRunSummary(`已提交全流程：${drink.name_zh} / ${shelfProfileLabel()} / 左臂第${state.selectedLayer}层位置${state.selectedPosition} → 右臂最终点`, false);
  try {
    const payload = await apiPost("/api/full-flow/execute", fullFlowRequestBody());
    renderStatus(payload);
    startPolling();
  } catch (error) {
    setRunSummary(`全流程启动失败：${error.message}`, true);
    if (error.payload) renderStatus(error.payload);
  } finally {
    await refreshStatus();
  }
}

async function stopExecution() {
  stopExecuteBtn.disabled = true;
  stopExecuteBtn.textContent = "停止中...";
  setRunSummary("正在停止当前执行...", false);
  try {
    const payload = await apiPost("/api/stop", {});
    setRunSummary(payload.message || "停止请求已发送", false);
    startPolling();
  } catch (error) {
    setRunSummary(`停止失败：${error.message}`, true);
    if (error.payload) renderStatus(error.payload);
  } finally {
    await refreshStatus();
  }
}

function recordingRequestBody() {
  const drink = selectedDrink();
  const taskMode = state.selectedTaskMode;
  const placeText = taskMode === "pick"
    ? (armSelect.value === "right" ? "return_to_fixed_start_holding" : "pick_place")
    : `layer_${state.selectedLayer}_position_${state.selectedPosition}`;
  const armText = taskMode === "full_flow" ? "left_right" : armSelect.value;
  const taskName = `${taskMode}_${drink?.id || "drink"}_${armText}`;
  const note = taskMode === "full_flow"
    ? `全流程 / ${shelfProfileLabel()} / ${drink?.name_zh || "-"} / 左臂${placeText} -> 右臂最终点`
    : `${taskModeLabel(taskMode)} / ${shelfProfileLabel()} / ${drink?.name_zh || "-"} / ${armLabel(armSelect.value)} / ${placeText}`;
  return {
    task_mode: taskMode,
    drink_id: drink?.id || "",
    arm: armSelect.value,
    task_name: taskName,
    note,
  };
}

async function startRecording() {
  await runRecordingRequest("start", recordingRequestBody());
}

async function executeAndRecordTask() {
  const drink = selectedDrink();
  if (!drink) {
    setRunSummary("请选择饮料", true);
    return;
  }

  state.combinedStarting = true;
  syncExecuteAvailability();
  if (state.selectedTaskMode === "full_flow") {
    const preview = await refreshFullFlowPreview();
    if (!preview?.ready) {
      const message = (preview?.issues || [])
        .map((issue) => issue.message || issue.code)
        .join("；") || "全流程预检未通过";
      setRunSummary(`无法启动全流程录制：${message}`, true);
      state.combinedStarting = false;
      syncExecuteAvailability();
      return;
    }
    const confirmed = window.confirm(
      `确认执行全流程并自动录制？\n\n货架：${shelfProfileLabel()} · Scene ${selectedShelfProfile()?.scene_id}\n饮料：${drink.name_zh}\n左臂：第${state.selectedLayer}层位置${state.selectedPosition}\n右臂：最终放置点 default\n\n录制就绪后才会启动机械臂；成功后自动校验并保存。`
    );
    if (!confirmed) {
      setRunSummary("已取消全流程录制", false);
      state.combinedStarting = false;
      syncExecuteAvailability();
      return;
    }
  }

  const taskMode = state.selectedTaskMode;
  const recordingBody = recordingRequestBody();
  const executionEndpoint = taskMode === "full_flow"
    ? "/api/full-flow/execute"
    : "/api/execute";
  const executionBody = taskMode === "full_flow"
    ? fullFlowRequestBody(true)
    : executeRequestBody(true);
  let recordingStarted = false;
  setRecordingBusy(true);
  setRunSummary("正在等待三路相机、双臂反馈和录包话题就绪…", false);
  try {
    const recordingPayload = await apiPost(
      "/api/recording/start",
      recordingBody,
    );
    recordingStarted = true;
    if (recordingPayload.recording) renderRecording(recordingPayload.recording);

    setRunSummary("录制已就绪，正在提交机械臂任务…", false);
    const payload = await apiPost(executionEndpoint, executionBody);
    statusOutput.textContent = JSON.stringify(payload, null, 2);
    setRunSummary(
      `已开始${taskModeLabel(taskMode)}并录制：${drink.name_zh}`,
      false,
    );
    startPolling();
  } catch (error) {
    if (recordingStarted) {
      try {
        await apiPost("/api/recording/stop", {});
        await apiPost("/api/recording/save", { mark: "launch_failed" });
      } catch (_cleanupError) {
        // The stopped/rejected session is intentionally kept for inspection.
      }
    }
    setRunSummary(`执行并录制启动失败：${error.message}`, true);
    if (error.payload?.recording) renderRecording(error.payload.recording);
  } finally {
    state.combinedStarting = false;
    setRecordingBusy(false);
    await refreshStatus();
  }
}

async function recordingAction(action) {
  await runRecordingRequest(action, {});
}

async function runRecordingRequest(action, body) {
  setRecordingBusy(true);
  try {
    await apiPost(`/api/recording/${action}`, body);
    await refreshStatus();
  } catch (error) {
    setRunSummary(`录制${recordingActionLabel(action)}失败：${error.message}`, true);
    if (error.payload?.recording) {
      renderRecording(error.payload.recording);
    }
  } finally {
    setRecordingBusy(false);
  }
}

function startPolling() {
  if (state.polling) return;
  state.polling = window.setInterval(refreshStatus, 1000);
}

function stopPolling() {
  if (!state.polling) return;
  window.clearInterval(state.polling);
  state.polling = null;
}

async function refreshStatus() {
  try {
    const payload = await apiGet("/api/status");
    renderStatus(payload);
  } catch (error) {
    setRunSummary(`状态读取失败：${error.message}`, true);
  }
}

function renderStatus(payload) {
  const status = payload || {};
  const ros = status.ros || {};
  const planning = status.planning || {};
  const recording = status.recording || {};
  rosStatus.textContent = ros.available
    ? `ROS ${ros.domain_id || "-"} · ${(planning.object_mode || "-").toUpperCase()} · ${ros.service || "-"}`
    : `ROS 不可用`;
  rosStatus.className = `status-chip ${ros.available ? "online" : "offline"}`;
  renderRecording(recording);

  const busy = Boolean(status.busy);
  state.busy = busy;
  syncExecuteAvailability();
  if (busy) {
    const job = status.current_job || status.job || {};
    const stopping = job.status === "stopping";
    stopExecuteBtn.disabled = stopping;
    stopExecuteBtn.textContent = stopping ? "停止中..." : "停止当前执行";
    if (job.recording_phase === "validating") {
      stopExecuteBtn.disabled = true;
      setRunSummary("机械臂任务已结束，正在停止 rosbag、校验并保存数据…", false);
    } else if ((job.task_mode || "restock") === "full_flow") {
      setRunSummary(stopping
        ? `正在停止全流程：${job.drink_name_zh || job.prompt || "-"} · ${fullFlowPhaseLabel(job.phase)}`
        : `全流程执行中：${job.drink_name_zh || job.prompt || "-"} · ${fullFlowPhaseLabel(job.phase)} · ${placeSummary(job)}`, false);
    } else {
      setRunSummary(stopping
        ? `正在停止${taskModeLabel(job.task_mode || "restock")}：${job.drink_name_zh || job.prompt || "-"}`
        : `执行中${taskModeLabel(job.task_mode || "restock")}：${job.drink_name_zh || job.prompt || "-"} / ${armLabel(job.arm)} / ${placeSummary(job)}`, false);
    }
    startPolling();
  } else {
    stopExecuteBtn.disabled = true;
    stopExecuteBtn.textContent = "停止当前执行";
    if (recordingActive(recording)) {
      startPolling();
    } else {
      stopPolling();
    }
    const last = status.last_result;
    if (last) {
      const ok = Boolean(last.ok || last.success);
      const prefix = ok ? "完成" : "失败";
      const failedPhase = last.task_mode === "full_flow" && last.failed_phase
        ? ` · ${fullFlowPhaseLabel(last.failed_phase)}`
        : "";
      const recorded = last.recording_result || null;
      const recordingSuffix = recorded?.ok
        ? ` · 录制已保存：${recorded.path || "完成"}`
        : (recorded
          ? ` · 录制未通过质量校验：${recorded.message || recorded.error || recorded.status}`
          : "");
      setRunSummary(
        `${prefix}${taskModeLabel(last.task_mode || "restock")}：${last.drink_name_zh || last.prompt || "-"}${failedPhase} · ${last.message || ""}${recordingSuffix}`,
        !ok || Boolean(recorded && !recorded.ok),
      );
    } else if (!statusOutput.textContent || statusOutput.textContent === "-") {
      setRunSummary("等待任务", false);
    }
  }
  statusOutput.textContent = JSON.stringify(status, null, 2);
  if (state.selectedTaskMode === "full_flow") {
    renderFullFlowReadiness();
  }
}

function fullFlowPhaseLabel(phase) {
  return {
    VALIDATING: "预检",
    LEFT_RESTOCK: "左臂上架阶段",
    TRANSITION_CHECK: "左右臂转场检查",
    RIGHT_PICK: "右臂拣货阶段",
    STOPPING: "停止中",
    STOPPED: "已停止",
    COMPLETED: "已完成",
    FAILED: "失败",
  }[phase] || phase || "准备中";
}

function renderRecording(recording) {
  state.lastRecording = recording || {};
  const enabled = recording.enabled !== false;
  const status = recording.status || "idle";
  const session = recording.session || null;
  recordStatusText.textContent = enabled
    ? recordingStatusLabel(status, session)
    : "录制未启用";
  recordStatusText.className = status === "error" ? "error" : "";

  const idle = status === "idle";
  const recordingNow = status === "starting" || status === "recording";
  const waitingDecision = status === "stopped" || status === "rejected" || status === "error";
  recordStartBtn.disabled = !enabled || !idle;
  recordStopBtn.disabled = !enabled || !recordingNow;
  recordSaveBtn.disabled = !enabled || !waitingDecision;
  recordDiscardBtn.disabled = !enabled || !waitingDecision;

  const lines = [];
  if (session?.episode_id) lines.push(`episode: ${session.episode_id}`);
  if (session?.elapsed_sec !== undefined) lines.push(`时长: ${formatDuration(session.elapsed_sec)}`);
  if (session?.target_arm) lines.push(`采集臂: ${session.target_arm}`);
  if (session?.ready_at) lines.push("门禁: 已就绪");
  if (session?.episode_started_at) lines.push("事件: START 已记录");
  if (session?.episode_ended_at) lines.push("事件: END/ABORT 已记录");
  if (session?.quality?.failures?.length) {
    lines.push(`质量失败: ${session.quality.failures.join("; ")}`);
  }
  if (session?.pending_path && waitingDecision) lines.push(`待处理: ${session.pending_path}`);
  if (session?.final_path && status === "saved") lines.push(`保存: ${session.final_path}`);
  if (!session && recording.last_saved?.path) lines.push(`最近保存: ${recording.last_saved.path}`);
  if (!session && recording.last_discarded?.episode_id) lines.push(`最近丢弃: ${recording.last_discarded.episode_id}`);
  if (recording.last_error) lines.push(`错误: ${recording.last_error}`);
  if (!lines.length) {
    lines.push(`目录: ${recording.target_dir || state.options.recording?.target_dir || "-"}`);
    lines.push(`格式: ${recording.storage || state.options.recording?.storage || "mcap"}`);
  }
  recordDetail.textContent = lines.join("\n");
}

function setRecordingBusy(isBusy) {
  if (!isBusy) {
    renderRecording(state.lastRecording || {});
    return;
  }
  [recordStartBtn, recordStopBtn, recordSaveBtn, recordDiscardBtn, executeRecordBtn].forEach((button) => {
    button.disabled = true;
  });
  executeBtn.disabled = true;
}

function recordingActive(recording) {
  return ["starting", "recording"].includes(recording?.status || "");
}

function recordingStatusLabel(status, session) {
  if (status === "starting") return `准备录制门禁 · ${formatDuration(session?.elapsed_sec || 0)}`;
  if (status === "recording") return `录制中 · ${formatDuration(session?.elapsed_sec || 0)}`;
  if (status === "stopped") return "已停止，等待保存";
  if (status === "rejected") return "质量验收失败，请丢弃或检查";
  if (status === "error") return "录制异常，等待处理";
  return "未录制";
}

function recordingActionLabel(action) {
  return {
    start: "开始",
    stop: "停止",
    save: "保存",
    discard: "丢弃",
  }[action] || action;
}

function formatDuration(seconds) {
  const total = Math.max(0, Math.floor(Number(seconds) || 0));
  const minutes = Math.floor(total / 60);
  const rest = total % 60;
  return `${minutes}:${String(rest).padStart(2, "0")}`;
}

function setRunSummary(message, isError = false) {
  runSummary.textContent = message;
  runSummary.className = isError ? "error" : "";
}

async function apiGet(path) {
  const response = await fetch(path, { cache: "no-store" });
  return parseResponse(response);
}

async function apiPost(path, body) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
    cache: "no-store",
  });
  return parseResponse(response);
}

async function parseResponse(response) {
  const text = await response.text();
  let payload = {};
  if (text) {
    try {
      payload = JSON.parse(text);
    } catch {
      payload = { raw: text };
    }
  }
  if (!response.ok) {
    const error = new Error(payload.error || payload.message || `${response.status} ${response.statusText}`);
    error.payload = payload;
    throw error;
  }
  return payload;
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}
