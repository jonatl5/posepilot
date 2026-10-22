(() => {
  'use strict';
  const canvas = document.getElementById('pose-canvas');
  const ctx = canvas.getContext('2d');
  const xInput = document.getElementById('target-x');
  const scaleInput = document.getElementById('target-scale');
  const playButton = document.getElementById('animate-demo');
  const occludeButton = document.getElementById('occlude-demo');
  const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)');
  let playing = false, occluded = false, animationFrame = null, phase = 0, lastTime = null;

  // A synthetic detection, not a camera feed, detector or learned controller.
  // Box size at s / s* = 1 is the size the object appears at the hover distance.
  const boxWidth = .26, boxHeight = .40;
  const signed = number => `${number >= 0 ? '+' : ''}${number.toFixed(2)}`;
  const arrived = (error, scale) => Math.abs(error) < .03 && Math.abs(scale - 1) < .04;

  function updateControls() {
    playButton.textContent = playing ? 'Pause approach' : 'Play approach';
    playButton.setAttribute('aria-pressed', String(playing));
    occludeButton.textContent = occluded ? 'Restore object' : 'Hide object';
    occludeButton.setAttribute('aria-pressed', String(occluded));
    xInput.setAttribute('aria-valuetext', `Horizontal error ${signed(Number(xInput.value)/100)}`);
    scaleInput.setAttribute('aria-valuetext', `Relative size ${(Number(scaleInput.value)/100).toFixed(2)} times`);
  }

  function roundedRect(x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y); ctx.lineTo(x + w - r, y); ctx.quadraticCurveTo(x + w, y, x + w, y + r);
    ctx.lineTo(x + w, y + h - r); ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
    ctx.lineTo(x + r, y + h); ctx.quadraticCurveTo(x, y + h, x, y + h - r);
    ctx.lineTo(x, y + r); ctx.quadraticCurveTo(x, y, x + r, y); ctx.closePath();
  }

  function draw() {
    if (!ctx) return;
    const width = 700, height = 560;
    const pixelScale = width / (canvas.clientWidth || width);
    const chartFont = size => `${Math.round(size * pixelScale)}px Consolas, monospace`;
    const error = Number(xInput.value)/100, scale = Number(scaleInput.value)/100;
    const done = !occluded && arrived(error, scale);
    const cx = (.5 + error) * width, cy = .52 * height;
    ctx.clearRect(0,0,width,height);
    ctx.fillStyle = '#f4f6fa'; ctx.fillRect(0,0,width,height);
    ctx.strokeStyle = '#e2e7ef'; ctx.lineWidth = 1;
    for (let x=0;x<=width;x+=35) {ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,height);ctx.stroke();}
    for (let y=0;y<=height;y+=35) {ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(width,y);ctx.stroke();}

    ctx.setLineDash([5,7]); ctx.strokeStyle = '#bcc9dd';
    ctx.beginPath();ctx.moveTo(width*.5,55);ctx.lineTo(width*.5,height-30);ctx.stroke();
    ctx.beginPath();ctx.moveTo(40,cy);ctx.lineTo(width-40,cy);ctx.stroke();
    // Goal: where the box sits when the drone is hovering in front of the object.
    ctx.strokeStyle = done ? '#2d6a12' : '#9fb0cc'; ctx.lineWidth = 1.4;
    ctx.strokeRect(width*(.5-boxWidth/2), cy-height*boxHeight/2, width*boxWidth, height*boxHeight);
    ctx.setLineDash([]);
    ctx.font=chartFont(14);ctx.fillStyle=done?'#2d6a12':'#7d8aa3';
    ctx.fillText('hover target',width*(.5-boxWidth/2),cy-height*boxHeight/2-8*pixelScale);
    ctx.font=chartFont(16);ctx.fillStyle='#52627e';
    ctx.fillText('0.0',16*pixelScale,height-14*pixelScale);ctx.textAlign='center';ctx.fillText('0.5',width*.5,height-14*pixelScale);ctx.textAlign='right';ctx.fillText('1.0',width-16*pixelScale,height-14*pixelScale);ctx.textAlign='left';

    const w = width*boxWidth*scale, h = height*boxHeight*scale, x = cx - w/2, y = cy - h/2;
    ctx.globalAlpha = occluded ? .14 : 1;
    // The object: a simple backpack shape inside its detection box.
    ctx.fillStyle = '#dfe5f0'; ctx.strokeStyle = '#627da9'; ctx.lineWidth = 2;
    roundedRect(x + w*.12, y + h*.10, w*.76, h*.86, w*.16); ctx.fill(); ctx.stroke();
    roundedRect(x + w*.24, y + h*.52, w*.52, h*.32, w*.08); ctx.stroke();
    ctx.beginPath(); ctx.arc(cx, y + h*.11, w*.16, Math.PI, 0); ctx.stroke();
    // Detection box and label.
    ctx.strokeStyle = done ? '#2d6a12' : '#1435ee'; ctx.lineWidth = 3; ctx.strokeRect(x, y, w, h);
    const tag = 'backpack 0.91';
    ctx.font = chartFont(15);
    const tagWidth = ctx.measureText(tag).width + 14*pixelScale;
    ctx.fillStyle = done ? '#2d6a12' : '#1435ee'; ctx.fillRect(x - 1.5, y - 24*pixelScale, tagWidth, 24*pixelScale);
    ctx.fillStyle = '#fff'; ctx.fillText(tag, x + 7*pixelScale, y - 7*pixelScale);
    // Horizontal error from the image center to the box center.
    ctx.strokeStyle='#1435ee';ctx.lineWidth=1.6;ctx.beginPath();ctx.moveTo(width*.5,cy);ctx.lineTo(cx,cy);ctx.stroke();
    ctx.fillStyle='#1435ee';ctx.beginPath();ctx.arc(cx,cy,5,0,Math.PI*2);ctx.fill();
    ctx.font=chartFont(16);
    const label = `e_x = ${signed(error)}`;
    const labelWidth = ctx.measureText(label).width;
    const labelX = x + w + 12*pixelScale + labelWidth > width-15*pixelScale ? x-labelWidth-12*pixelScale : x+w+12*pixelScale;
    ctx.fillText(label,labelX,cy-12*pixelScale);
    ctx.globalAlpha=1;
    if(occluded){
      ctx.fillStyle='#f4f6faf0';ctx.fillRect(20*pixelScale,cy-48*pixelScale,width-40*pixelScale,100*pixelScale);
      ctx.fillStyle='#984526';ctx.textAlign='center';ctx.font=`${Math.round(18*pixelScale)}px "Segoe UI", sans-serif`;
      ctx.fillText('Object not detected',width*.5,cy-8*pixelScale);
      ctx.font=chartFont(16);ctx.fillStyle='#52627e';ctx.fillText('HOLD / SEARCH',width*.5,cy+20*pixelScale);ctx.textAlign='left';
    }
    const state = document.getElementById('state-output');
    document.getElementById('x-output').textContent=occluded?'—':signed(error);
    document.getElementById('scale-output').textContent=occluded?'—':`${scale.toFixed(2)}×`;
    state.textContent=occluded?'Lost':done?'Arrived':'Approaching';
    state.style.color=occluded?'#a45135':done?'#2d6a12':'#1435ee';
    updateControls();
  }

  // Play loops one approach: start off-center and far away, close in, hover, repeat.
  function animate(time){
    if(!playing)return;
    if(lastTime!==null)phase+=Math.min((time-lastTime)/1000,.1);
    lastTime=time;
    const t=(phase%7)/7, progress=Math.min(t/.7,1), eased=1-Math.pow(1-progress,3);
    xInput.value=Math.round(22*(1-eased));
    scaleInput.value=Math.round(70+30*eased);
    draw();animationFrame=requestAnimationFrame(animate);
  }
  function stop(){playing=false;cancelAnimationFrame(animationFrame);animationFrame=null;lastTime=null;updateControls();draw();}
  xInput.addEventListener('input',()=>{stop();draw();});
  scaleInput.addEventListener('input',()=>{stop();draw();});
  playButton.addEventListener('click',()=>{if(playing)stop();else{playing=true;phase=0;lastTime=null;updateControls();animationFrame=requestAnimationFrame(animate);}});
  occludeButton.addEventListener('click',()=>{occluded=!occluded;draw();});
  document.addEventListener('visibilitychange',()=>{if(document.hidden&&playing)stop();});
  reducedMotion.addEventListener('change',()=>{if(reducedMotion.matches)stop();});

  const navigationLinks=[...document.querySelectorAll('.nav a')].filter(link=>link.hash);
  if('IntersectionObserver' in window){
    const observer=new IntersectionObserver(entries=>{
      entries.forEach(entry=>{if(entry.isIntersecting){navigationLinks.forEach(link=>{const active=link.hash===`#${entry.target.id}`;link.classList.toggle('active',active);if(active)link.setAttribute('aria-current','location');else link.removeAttribute('aria-current');});}});
    },{rootMargin:'-20% 0px -60% 0px'});
    navigationLinks.forEach(link=>{const section=document.querySelector(link.hash);if(section)observer.observe(section);});
    observer.observe(document.getElementById('top'));
  }
  updateControls();
  draw();
  if('ResizeObserver' in window){new ResizeObserver(draw).observe(canvas);}
})();
