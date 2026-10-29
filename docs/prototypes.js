(() => {
  'use strict';
  const x = document.getElementById('lab-x');
  const distance = document.getElementById('lab-range');
  const age = document.getElementById('lab-age');
  const visible = document.getElementById('lab-visible');
  const tof = document.getElementById('lab-tof');
  const target = document.getElementById('lab-target');
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  function update() {
    const ex = Number(x.value) / 100;
    const metres = Number(distance.value) / 100;
    const seconds = Number(age.value) / 1000;
    const rangeValid = tof.checked && metres <= 1.2;
    let forward = 0, yaw = 0, state = 'HOLD';
    let reason = 'Target missing or observation at least 0.35 s old: zero all channels.';
    if (visible.checked && seconds < .35) {
      if (rangeValid && metres <= .4) {
        reason = 'Close raw range overrides target association.';
      } else if (metres <= 1.15 && !rangeValid) {
        reason = 'Near approach requires a fresh forward sample.';
      } else if (rangeValid && Math.abs(ex) < .05 && Math.abs(metres - .65) < .08) {
        state = 'ARRIVAL HOLD';
        reason = 'Within measured tolerances. Python requires a continuous two-second hold.';
      } else {
        yaw = Math.round(25 * clamp(3.5 * ex, -1, 1));
        if (Math.abs(ex) > .12) {
          state = 'CENTER';
          reason = 'Target off-center: yaw only.';
        } else {
          let normalized = clamp(1.5 * (metres - .65), 0, 1);
          if (rangeValid) normalized = Math.min(normalized, Math.max(0, (metres - .4) / .35));
          forward = Math.round(20 * normalized);
          state = 'APPROACH';
          reason = 'Centered target: bounded forward approach. No automatic reverse.';
        }
      }
    }
    document.getElementById('lab-x-value').textContent = `${ex >= 0 ? '+' : ''}${ex.toFixed(2)}`;
    document.getElementById('lab-range-value').textContent = `${metres.toFixed(2)} m`;
    document.getElementById('lab-age-value').textContent = `${seconds.toFixed(2)} s`;
    document.getElementById('lab-command').textContent = `rc 0 ${forward} 0 ${yaw}`;
    document.getElementById('lab-state').textContent = state;
    document.getElementById('lab-reason').textContent = reason;
    target.style.left = `${50 + ex * 100}%`;
    target.style.width = `${16.5 / metres}%`;
    target.style.opacity = visible.checked ? '1' : '.12';
    document.getElementById('lab-stage-note').textContent = 'Illustration only · no drone connection';
  }
  [x, distance, age, visible, tof].forEach(input => input.addEventListener('input', update));
  const presets = {align: [25, 180, true], approach: [0, 180, true], hover: [0, 65, true], lost: [0, 180, false]};
  document.querySelectorAll('[data-preset]').forEach(button => button.addEventListener('click', () => {
    const preset = presets[button.dataset.preset];
    x.value = preset[0]; distance.value = preset[1]; visible.checked = preset[2]; age.value = 100; tof.checked = true;
    update();
  }));
  update();
})();
