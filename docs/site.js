// Chart tooltips. Every value is also in a direct label or a table view; this only adds hover and focus.
(() => {
  const tip = document.createElement('div');
  tip.className = 'tip';
  tip.setAttribute('role', 'status');
  document.body.appendChild(tip);
  const show = (el, x, y) => {
    const [value, ...rest] = (el.getAttribute('data-tip') || '').split('|');
    tip.replaceChildren();
    const strong = document.createElement('strong');
    strong.textContent = value;
    tip.appendChild(strong);
    for (const line of rest) {
      const row = document.createElement('div');
      row.textContent = line;
      tip.appendChild(row);
    }
    const pad = 14, w = tip.offsetWidth, h = tip.offsetHeight;
    tip.style.left = Math.min(window.innerWidth - w - 8, x + pad) + 'px';
    tip.style.top = Math.max(8, y - h - pad) + 'px';
    tip.classList.add('on');
  };
  const hide = () => tip.classList.remove('on');
  for (const el of document.querySelectorAll('[data-tip]')) {
    el.setAttribute('tabindex', '0');
    el.addEventListener('pointermove', e => show(el, e.clientX, e.clientY));
    el.addEventListener('pointerleave', hide);
    el.addEventListener('focus', () => {
      const r = el.getBoundingClientRect();
      show(el, r.left + r.width / 2, r.top);
    });
    el.addEventListener('blur', hide);
  }
})();
