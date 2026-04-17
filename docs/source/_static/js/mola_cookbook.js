// MOLA ROS 2 cookbook — live parameter editor.
//
// Scans every `.mola-tpl` container on the page, remembers its original
// command text, and re-renders it whenever the user tweaks the form at the
// top of the page. Values persist in localStorage.

(function () {
  'use strict';

  const STORAGE_KEY = 'mola_cookbook_params_v1';

  const FIELDS = [
    { key: 'LIDAR_TOPIC', label: 'LiDAR topic',       def: '/ouster/points' },
    { key: 'IMU_TOPIC',   label: 'IMU topic',         def: '/imu' },
    { key: 'GNSS_TOPIC',  label: 'GNSS topic',        def: '/gps' },
    { key: 'ODOM_TOPIC',  label: 'External odom topic', def: '/wheel_odom' },
    { key: 'BASE_LINK',   label: 'base_link frame',   def: 'base_link' },
    { key: 'NS',          label: 'ROS 2 namespace',   def: 'robot1' },
    { key: 'BAG_PATH',    label: 'rosbag path',       def: '/path/to/your.mcap' },
    { key: 'MM_PATH',     label: '.mm map path',      def: '/path/to/map.mm' },
  ];

  function loadValues() {
    let stored = {};
    try {
      stored = JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}');
    } catch (_) {
      stored = {};
    }
    const values = {};
    for (const f of FIELDS) {
      values[f.key] = (stored[f.key] !== undefined) ? stored[f.key] : f.def;
    }
    return values;
  }

  function saveValues(values) {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(values));
    } catch (_) { /* quota / disabled — silently ignore */ }
  }

  function substitute(template, values) {
    let out = template;
    for (const [k, v] of Object.entries(values)) {
      // Replace __KEY__ globally.
      out = out.split('__' + k + '__').join(v);
    }
    return out;
  }

  function captureTemplates() {
    // Find every <pre> inside a .mola-tpl container and capture its text
    // once, so subsequent substitutions start from the pristine template.
    const nodes = [];
    document.querySelectorAll('.mola-tpl pre').forEach((pre) => {
      if (!pre.dataset.molaTpl) {
        pre.dataset.molaTpl = pre.textContent;
      }
      nodes.push(pre);
    });
    return nodes;
  }

  function renderAll(nodes, values) {
    for (const pre of nodes) {
      pre.textContent = substitute(pre.dataset.molaTpl, values);
    }
  }

  function buildForm(container, values, nodes) {
    container.innerHTML = '';
    const title = document.createElement('p');
    title.className = 'mola-cookbook-hint';
    title.textContent = 'Edit any field — all command blocks below update and persist in your browser.';
    container.appendChild(title);

    const grid = document.createElement('div');
    grid.className = 'mola-cookbook-grid';

    for (const f of FIELDS) {
      const row = document.createElement('label');
      row.className = 'mola-cookbook-row';

      const lbl = document.createElement('span');
      lbl.className = 'mola-cookbook-label';
      lbl.textContent = f.label;

      const input = document.createElement('input');
      input.type = 'text';
      input.value = values[f.key];
      input.placeholder = f.def;
      input.spellcheck = false;
      input.autocomplete = 'off';
      input.dataset.key = f.key;
      input.addEventListener('input', () => {
        values[f.key] = input.value || f.def;
        saveValues(values);
        renderAll(nodes, values);
      });

      row.appendChild(lbl);
      row.appendChild(input);
      grid.appendChild(row);
    }

    const reset = document.createElement('button');
    reset.type = 'button';
    reset.className = 'mola-cookbook-reset';
    reset.textContent = 'Reset to defaults';
    reset.addEventListener('click', () => {
      for (const f of FIELDS) {
        values[f.key] = f.def;
      }
      saveValues(values);
      renderAll(nodes, values);
      // refresh inputs
      container.querySelectorAll('input[data-key]').forEach((i) => {
        i.value = values[i.dataset.key];
      });
    });

    container.appendChild(grid);
    container.appendChild(reset);
  }

  function init() {
    const formContainer = document.getElementById('mola-cookbook-form');
    if (!formContainer) return; // not the cookbook page

    const values = loadValues();
    const nodes = captureTemplates();
    buildForm(formContainer, values, nodes);
    renderAll(nodes, values);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
