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

  // A synthetic landmark chart, not a camera feed or a learned controller.
  // Indices follow the MediaPipe Pose landmark convention.
  const landmarks = [
    [0,-.242],[-.021,-.254],[-.028,-.255],[-.035,-.253],[.021,-.254],[.028,-.255],[.035,-.253],[-.055,-.243],[.055,-.243],[-.018,-.219],[.018,-.219],
    [-.085,-.08],[.085,-.08],[-.123,.025],[.126,.025],[-.15,.126],[.159,.116],[-.155,.152],[.173,.143],[-.146,.15],[.165,.144],[-.14,.13],[.153,.126],
    [-.057,.08],[.057,.08],[-.065,.217],[.077,.213],[-.086,.346],[.09,.347],[-.095,.356],[.082,.357],[-.062,.37],[.117,.37]
  ];
  const connections = [[0,1],[1,2],[2,3],[3,7],[0,4],[4,5],[5,6],[6,8],[9,10],[11,12],[11,13],[13,15],[15,17],[15,19],[15,21],[17,19],[12,14],[14,16],[16,18],[16,20],[16,22],[18,20],[11,23],[12,24],[23,24],[23,25],[24,26],[25,27],[26,28],[27,29],[29,31],[27,31],[28,30],[30,32],[28,32]];
  const torso = new Set([11,12,23,24]);
  const signed = number => `${number >= 0 ? '+' : ''}${number.toFixed(2)}`;

  function updateControls() {
    playButton.textContent = playing ? 'Pause motion' : 'Play motion';
    playButton.setAttribute('aria-pressed', String(playing));
    occludeButton.textContent = occluded ? 'Restore target' : 'Occlude target';
    occludeButton.setAttribute('aria-pressed', String(occluded));
    xInput.setAttribute('aria-valuetext', `Horizontal error ${signed(Number(xInput.value)/100)}`);
    scaleInput.setAttribute('aria-valuetext', `Relative scale ${(Number(scaleInput.value)/100).toFixed(2)} times`);
  }

  function draw() {
    if (!ctx) return;
    const width = 700, height = 560;
    const pixelScale = width / (canvas.clientWidth || width);
    const chartFont = size => `${Math.round(size * pixelScale)}px Consolas, monospace`;
    const error = Number(xInput.value)/100, scale = Number(scaleInput.value)/100;
    const centerX = .5 + error, centerY = .5;
    ctx.clearRect(0,0,width,height);
    ctx.fillStyle = '#f4f6fa'; ctx.fillRect(0,0,width,height);
    ctx.strokeStyle = '#e2e7ef'; ctx.lineWidth = 1;
    for (let x=0;x<=width;x+=35) {ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,height);ctx.stroke();}
    for (let y=0;y<=height;y+=35) {ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(width,y);ctx.stroke();}

    ctx.setLineDash([5,7]); ctx.strokeStyle = '#bcc9dd';
    ctx.beginPath();ctx.moveTo(width*.5,55);ctx.lineTo(width*.5,height-30);ctx.stroke();
    ctx.beginPath();ctx.moveTo(40,height*centerY);ctx.lineTo(width-40,height*centerY);ctx.stroke();
    ctx.setLineDash([]);
    ctx.strokeStyle='#b6c2d5';ctx.lineWidth=1.4;
    ctx.beginPath();ctx.arc(width*.5,height*centerY,9,0,Math.PI*2);ctx.stroke();
    ctx.font=chartFont(16);ctx.fillStyle='#52627e';
    ctx.fillText('0.0',16*pixelScale,height-14*pixelScale);ctx.textAlign='center';ctx.fillText('0.5',width*.5,height-14*pixelScale);ctx.textAlign='right';ctx.fillText('1.0',width-16*pixelScale,height-14*pixelScale);ctx.textAlign='left';

    const points=landmarks.map(([x,y],index)=>{
      const sway=playing&&!torso.has(index)&&index>12 ? Math.sin(phase*3+index*.7)*.009 : 0;
      return [(centerX+(x+sway)*scale)*width,(centerY+y*scale)*height];
    });
    ctx.globalAlpha=occluded?.14:1;
    ctx.fillStyle='#1435ee0a';
    ctx.beginPath();[11,12,24,23].forEach((index,i)=>i?ctx.lineTo(...points[index]):ctx.moveTo(...points[index]));ctx.closePath();ctx.fill();
    ctx.strokeStyle='#627da9';ctx.lineWidth=2.1;
    connections.forEach(([a,b])=>{ctx.beginPath();ctx.moveTo(...points[a]);ctx.lineTo(...points[b]);ctx.stroke();});
    ctx.strokeStyle='#1435ee';ctx.lineWidth=3;
    [[11,12],[12,24],[24,23],[23,11]].forEach(([a,b])=>{ctx.beginPath();ctx.moveTo(...points[a]);ctx.lineTo(...points[b]);ctx.stroke();});
    points.forEach(([x,y],index)=>{
      ctx.beginPath();ctx.fillStyle=torso.has(index)?'#1435ee':'#f4f6fa';ctx.strokeStyle=torso.has(index)?'#1435ee':'#7388ab';
      ctx.lineWidth=1.5;ctx.arc(x,y,torso.has(index)?5:3,0,Math.PI*2);ctx.fill();ctx.stroke();
    });
    const cx=centerX*width,cy=centerY*height;
    ctx.strokeStyle='#1435ee';ctx.lineWidth=1.6;ctx.beginPath();ctx.moveTo(width*.5,cy);ctx.lineTo(cx,cy);ctx.stroke();
    ctx.fillStyle='#1435ee';ctx.beginPath();ctx.arc(cx,cy,5,0,Math.PI*2);ctx.fill();
    ctx.setLineDash([4,5]);ctx.strokeStyle='#6382e4';ctx.lineWidth=1;
    ctx.strokeRect(cx-.196*width*scale,cy-.29*height*scale,.392*width*scale,.70*height*scale);ctx.setLineDash([]);
    ctx.font=chartFont(16);ctx.fillStyle='#1435ee';
    const label = `e_x = ${signed(error)}`;
    const labelWidth = ctx.measureText(label).width;
    const labelX = cx + 14*pixelScale + labelWidth > width-15*pixelScale ? cx-labelWidth-14*pixelScale : cx+14*pixelScale;
    ctx.fillText(label,labelX,cy-18*pixelScale);
    if(canvas.clientWidth>=500){
      ctx.font=chartFont(16);ctx.fillStyle='#52627e';
      ctx.fillText('11',points[11][0]-35*pixelScale,points[11][1]-12*pixelScale);ctx.fillText('12',points[12][0]+10*pixelScale,points[12][1]-12*pixelScale);
      ctx.fillText('23',points[23][0]-35*pixelScale,points[23][1]+23*pixelScale);ctx.fillText('24',points[24][0]+10*pixelScale,points[24][1]+23*pixelScale);
    }
    ctx.globalAlpha=1;
    if(occluded){
      ctx.fillStyle='#f4f6faf0';ctx.fillRect(20*pixelScale,cy-48*pixelScale,width-40*pixelScale,100*pixelScale);
      ctx.fillStyle='#984526';ctx.textAlign='center';ctx.font=`${Math.round(18*pixelScale)}px "Segoe UI", sans-serif`;
      ctx.fillText('Target temporarily lost',width*.5,cy-8*pixelScale);
      ctx.font=chartFont(16);ctx.fillStyle='#52627e';ctx.fillText('HOLD / HANDOVER',width*.5,cy+20*pixelScale);ctx.textAlign='left';
    }
    document.getElementById('x-output').textContent=occluded?'—':signed(error);
    document.getElementById('scale-output').textContent=occluded?'—':`${scale.toFixed(2)}×`;
    document.getElementById('state-output').textContent=occluded?'Lost':'Valid';
    document.getElementById('state-output').style.color=occluded?'#a45135':'#1435ee';
    updateControls();
  }

  function animate(time){
    if(!playing)return;
    if(lastTime!==null)phase+=Math.min((time-lastTime)/1000,.1);
    lastTime=time;
    xInput.value=Math.round(Math.sin(phase*.8)*23);
    scaleInput.value=Math.round(100+Math.sin(phase*.52)*16);
    draw();animationFrame=requestAnimationFrame(animate);
  }
  function stop(){playing=false;cancelAnimationFrame(animationFrame);animationFrame=null;lastTime=null;updateControls();draw();}
  xInput.addEventListener('input',()=>{stop();draw();});
  scaleInput.addEventListener('input',()=>{stop();draw();});
  playButton.addEventListener('click',()=>{if(playing)stop();else{playing=true;lastTime=null;updateControls();animationFrame=requestAnimationFrame(animate);}});
  occludeButton.addEventListener('click',()=>{occluded=!occluded;draw();});
  document.addEventListener('visibilitychange',()=>{if(document.hidden&&playing)stop();});
  reducedMotion.addEventListener('change',()=>{if(reducedMotion.matches)stop();});

  const navigationLinks=[...document.querySelectorAll('.nav a')];
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
