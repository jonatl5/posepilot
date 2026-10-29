/* Two explainers: a rule illustration and playback of recorded Python output.
   Neither runs a detector, policy, or flight connection in the browser. */
const $ = (id) => document.getElementById(id);
const colors = { blue: '#1435ee', ink: '#18243a', muted: '#536680', line: '#d9e1ef', red: '#ad402c', green: '#15714f' };

function line(ctx, x1, y1, x2, y2, color, width = 1, dash = []) {
  ctx.save();
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash);
  ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke(); ctx.restore();
}
function label(ctx, text, x, y, color = colors.muted, size = 18, align = 'left') {
  ctx.fillStyle = color; ctx.font = `500 ${size}px "Segoe UI", sans-serif`; ctx.textAlign = align; ctx.fillText(text, x, y);
}

function supervisorExplorer() {
  const canvas = $('object-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  let invalid = false, hidden = false, nearRequired = false;
  function render() {
    const ex = Number($('target-x').value) / 100;
    const distance = Number($('wall-distance').value) / 100;
    const valid = !invalid && distance <= 1.2;
    nearRequired ||= valid || distance <= 1.15;
    let state = 'Approach', reason = 'Centered target · bounded forward command', accent = colors.blue;
    if (hidden) { state = 'Hold'; reason = 'Target missing · zero motion'; accent = colors.red; }
    else if (valid && distance <= .4) { state = 'Stop'; reason = 'Minimum clearance reached · zero motion'; accent = colors.red; }
    else if (nearRequired && !valid) { state = 'Hold'; reason = 'Near-range reading unavailable · zero motion'; accent = colors.red; }
    else if (Math.abs(ex) > .12) { state = 'Center'; reason = 'Align first · forward command held at zero'; }
    else if (distance <= .65) { state = 'Hold'; reason = 'At or inside the desired stand-off'; accent = colors.green; }
    $('x-output').textContent = hidden ? '—' : `${ex >= 0 ? '+' : ''}${ex.toFixed(2)}`;
    $('range-output').textContent = valid ? `${distance.toFixed(2)} m` : 'Invalid';
    $('state-output').textContent = state;
    $('state-output').style.color = accent;
    $('target-x').setAttribute('aria-valuetext', `${ex.toFixed(2)} normalized centering error`);
    $('wall-distance').setAttribute('aria-valuetext', `${distance.toFixed(2)} meters`);
    $('sensor-toggle').setAttribute('aria-pressed', String(invalid));
    $('sensor-toggle').textContent = invalid ? 'Restore ToF' : 'Invalidate ToF';
    $('occlude-demo').setAttribute('aria-pressed', String(hidden));
    $('occlude-demo').textContent = hidden ? 'Show target' : 'Hide target';
    canvas.setAttribute('aria-label', `Supervisor illustration: ${state}. ${reason}. Wall distance ${distance.toFixed(2)} meters. Forward ToF ${valid ? 'valid' : 'invalid'}.`);

    ctx.clearRect(0, 0, 700, 500); ctx.fillStyle = '#eff3fa'; ctx.fillRect(0, 0, 700, 500);
    for (let x = 0; x < 700; x += 50) line(ctx, x, 0, x, 500, '#e0e7f2');
    for (let y = 0; y < 500; y += 50) line(ctx, 0, y, 700, y, '#e0e7f2');
    label(ctx, 'FRONT CAMERA · PRINTED TARGET', 24, 34, colors.muted, 18);
    label(ctx, 'SCHEMATIC', 676, 34, colors.muted, 16, 'right');
    const cx = 350 + ex * 700, cy = 241;
    const size = Math.min(275, Math.max(135, 210 / distance));
    if (!hidden) {
      ctx.fillStyle = '#cbd5e51f'; ctx.fillRect(cx - size / 2 + 8, cy - size / 2 + 8, size, size);
      ctx.fillStyle = '#fff'; ctx.fillRect(cx - size / 2, cy - size / 2, size, size);
      ctx.save(); ctx.translate(cx, cy); ctx.scale(size / 240, size / 240);
      // A simple drawn backpack stands for the object printed on the poster.
      ctx.strokeStyle = '#8798b2'; ctx.lineWidth = 10;
      ctx.beginPath(); ctx.roundRect(-23, -85, 46, 37, 15); ctx.stroke();
      ctx.fillStyle = '#768aaa'; ctx.beginPath(); ctx.roundRect(-63, -64, 126, 147, 26); ctx.fill();
      ctx.fillStyle = '#526b92'; ctx.beginPath(); ctx.roundRect(-49, 7, 98, 59, 13); ctx.fill();
      line(ctx, -36, 24, 36, 24, '#d7e1f3', 3); line(ctx, -48, -41, 48, -41, '#bac8de', 3);
      ctx.restore();
      const padding = 9;
      ctx.strokeStyle = colors.blue; ctx.lineWidth = 3;
      ctx.strokeRect(cx - size / 2 - padding, cy - size / 2 - padding, size + padding * 2, size + padding * 2);
      label(ctx, 'OBJECT BOX', cx - size / 2, cy - size / 2 - 22, colors.blue, 18);
      line(ctx, 350, cy, cx, cy, colors.blue, 2, [5, 5]);
      ctx.beginPath(); ctx.arc(cx, cy, 5, 0, Math.PI * 2); ctx.fillStyle = colors.blue; ctx.fill();
    } else {
      label(ctx, 'NO CONFIRMED TARGET', 350, cy, colors.muted, 24, 'center');
    }
    line(ctx, 350, 63, 350, 407, '#92a1b9', 1.5, [6, 7]);
    line(ctx, 337, cy, 363, cy, colors.ink, 2); line(ctx, 350, cy - 13, 350, cy + 13, colors.ink, 2);
    ctx.fillStyle = '#fff'; ctx.fillRect(0, 432, 700, 68);
    ctx.fillStyle = accent; ctx.fillRect(0, 432, 5, 68);
    label(ctx, reason, 24, 461, accent, 20);
    label(ctx, 'Illustrative readings · reset clears the near-range latch', 24, 486, colors.muted, 16);
  }
  $('target-x').addEventListener('input', render);
  $('wall-distance').addEventListener('input', render);
  $('sensor-toggle').addEventListener('click', () => { invalid = !invalid; render(); });
  $('occlude-demo').addEventListener('click', () => { hidden = !hidden; render(); });
  $('reset-demo').addEventListener('click', () => {
    invalid = hidden = nearRequired = false;
    $('target-x').value = '12'; $('wall-distance').value = '90'; render();
  });
  render();
}

async function recordedRollout() {
  const canvas = $('rollout-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const position = $('rollout-position'), play = $('rollout-play'), select = $('controller-select');
  let data, index = 0, playing = false, frame = null, lastTime = null;
  function pause() {
    playing = false; cancelAnimationFrame(frame); lastTime = null;
    play.textContent = 'Play rollout'; play.setAttribute('aria-pressed', 'false');
  }
  function render() {
    const run = data.controllers[select.value], samples = run.samples, sample = samples[index];
    position.max = String(samples.length - 1); position.value = String(index);
    position.setAttribute('aria-valuetext', `${sample.t.toFixed(1)} seconds of recorded simulation`);
    $('rollout-time').textContent = `${sample.t.toFixed(1)} s / ${samples.at(-1).t.toFixed(1)} s`;
    $('rollout-distance').textContent = `${(-sample.x).toFixed(2)} m`;
    $('rollout-action').textContent = `${sample.forward.toFixed(2)} / ${sample.yaw.toFixed(2)}`;
    $('rollout-outcome').textContent = run.outcome === 'success' ? 'Success (recorded)' : 'Timeout (recorded)';
    const reason = sample.reason.replaceAll('_', ' ');
    const status = `Seed ${data.seed} · ${reason}. Actions are normalized; clearance is simulator ground truth.`;
    if ($('rollout-status').textContent !== status) $('rollout-status').textContent = status;
    canvas.setAttribute('aria-label', `Recorded ${select.value.toUpperCase()} simulation at ${sample.t.toFixed(1)} seconds: wall clearance ${(-sample.x).toFixed(2)} meters. Whole episode outcome: ${run.outcome}.`);
    const minX = Math.min(-3.1, ...samples.map(s => s.x)) - .2;
    const maxY = Math.max(.85, ...samples.map(s => Math.abs(s.y))) + .15;
    const px = x => 62 + (x - minX) / (-minX) * 664;
    const py = y => 225 - y / maxY * 138;
    const wall = px(0);
    ctx.clearRect(0, 0, 800, 440); ctx.fillStyle = '#f6f8fc'; ctx.fillRect(0, 0, 800, 440);
    ctx.fillStyle = '#fbeae6'; ctx.fillRect(px(-data.minimum_m), 67, wall - px(-data.minimum_m), 302);
    for (let m = Math.ceil(minX); m <= 0; m++) {
      line(ctx, px(m), 67, px(m), 369, colors.line);
      label(ctx, `${-m} m`, px(m), 395, colors.muted, 17, 'center');
    }
    for (const y of [-.5, 0, .5]) line(ctx, 62, py(y), wall, py(y), colors.line);
    line(ctx, px(-data.stand_off_m), 67, px(-data.stand_off_m), 369, colors.green, 2, [6, 6]);
    line(ctx, px(-data.minimum_m), 67, px(-data.minimum_m), 369, colors.red, 2, [3, 5]);
    line(ctx, wall, 67, wall, 369, '#526580', 7);
    line(ctx, wall, py(.15), wall, py(-.15), colors.blue, 12);
    label(ctx, 'TOP VIEW', 25, 32, colors.ink, 18);
    label(ctx, 'Stand-off 0.65 m', 235, 32, colors.green, 18);
    label(ctx, 'Minimum 0.40 m', 470, 32, colors.red, 18);
    label(ctx, 'Wall', wall, 55, colors.ink, 18, 'center');
    ctx.beginPath();
    samples.slice(0, index + 1).forEach((s, i) => i ? ctx.lineTo(px(s.x), py(s.y)) : ctx.moveTo(px(s.x), py(s.y)));
    ctx.strokeStyle = colors.blue; ctx.lineWidth = 3; ctx.stroke();
    ctx.beginPath(); ctx.arc(px(samples[0].x), py(samples[0].y), 5, 0, Math.PI * 2); ctx.fillStyle = '#7e8caa'; ctx.fill();
    ctx.save(); ctx.translate(px(sample.x), py(sample.y)); ctx.rotate(-sample.heading);
    ctx.fillStyle = colors.blue; ctx.strokeStyle = '#fff'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(17, 0); ctx.lineTo(-12, -10); ctx.lineTo(-7, 0); ctx.lineTo(-12, 10); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
    label(ctx, 'Wall clearance →', 390, 427, colors.muted, 17, 'center');
  }
  function tick(time) {
    if (!playing) return;
    if (lastTime === null) lastTime = time;
    const steps = Math.floor((time - lastTime) / (data.dt * 1000));
    if (steps > 0) {
      index = Math.min(index + steps, data.controllers[select.value].samples.length - 1);
      lastTime += steps * data.dt * 1000; render();
      if (index >= Number(position.max)) { pause(); return; }
    }
    frame = requestAnimationFrame(tick);
  }
  try {
    const response = await fetch('./evidence/trajectory.json');
    if (!response.ok) throw new Error('Trace unavailable');
    data = await response.json();
    if (!data.controllers?.pid?.samples?.length || !data.controllers?.ppo?.samples?.length) throw new Error('Invalid trace');
    render(); position.disabled = play.disabled = false;
    select.addEventListener('change', () => { pause(); index = 0; render(); });
    position.addEventListener('input', () => { pause(); index = Number(position.value); render(); });
    play.addEventListener('click', () => {
      if (playing) { pause(); return; }
      if (index >= Number(position.max)) index = 0;
      playing = true; play.textContent = 'Pause rollout'; play.setAttribute('aria-pressed', 'true');
      render(); frame = requestAnimationFrame(tick);
    });
    document.addEventListener('visibilitychange', () => { if (document.hidden) pause(); });
    // Playback is always opt-in, including when reduced motion is requested.
  } catch {
    select.disabled = true;
    $('rollout-status').textContent = 'The recorded trace could not load. Results remain available below; try reloading this page.';
    label(ctx, 'Recorded trace unavailable', 400, 200, colors.muted, 25, 'center');
  }
}

supervisorExplorer();
recordedRollout();
const navLinks = [...document.querySelectorAll('.nav a[href^="#"]')];
if ('IntersectionObserver' in window) {
  const observer = new IntersectionObserver(entries => {
    for (const entry of entries) if (entry.isIntersecting) {
      navLinks.forEach(a => {
        const active = a.hash === `#${entry.target.id}`;
        a.classList.toggle('active', active);
        if (active) a.setAttribute('aria-current', 'location'); else a.removeAttribute('aria-current');
      });
    }
  }, { rootMargin: '-15% 0px -65% 0px' });
  navLinks.forEach(a => { const section = document.querySelector(a.hash); if (section) observer.observe(section); });
}
