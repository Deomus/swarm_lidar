'use strict';
const $ = id => document.getElementById(id);
const names = ['Ведущий', 'Ведомый 1', 'Ведомый 2', 'Ведомый 3'];
const shapes = {line:'Линия',column:'Колонна',wedge:'Клин'};
const phases = {idle:'Ожидание полёта',starting:'Подготовка и взлёт',running:'Выполняется сценарий',ready:'Группа готова к командам',landing:'Выполняется посадка',complete:'Полёт завершён',cancelled:'Полёт прерван, посадка выполнена',error:'Проверьте журнал полёта'};
const modes = {FOLLOWING:'Строй установлен',FORMATION_CHANGE:'Перестроение',DATA_HOLD:'Ожидание измерений',DISABLED:'Управление строем выключено',HOLD:'Удержание',COLLISION_STOP:'Остановка при сближении',CLOCK_RESET:'Сброс времени'};
let current = null, sending = false, localError = '', target = null;
let view = {cx:0, cy:0, scale:12, ready:false}, dragging = null;
// Те же ориентиры, что и в учебной площадке Gazebo: ось X красная,
// ось Y синяя, цилиндры находятся на расстоянии 25 м от начала координат.
const groundMarks = [
  {x:25, y:0, color:'#f26114', label:'X +25'},
  {x:-25, y:0, color:'#a638cc', label:'X −25'},
  {x:0, y:25, color:'#1fa647', label:'Y +25'},
  {x:0, y:-25, color:'#f2c714', label:'Y −25'},
];
const token = document.querySelector('meta[name=panel-token]').content;
const rows = names.map((name, i) => {
  const tr = document.createElement('tr');
  for (const text of [name,'Нет связи','—','—']) { const td=document.createElement('td');td.textContent=text;tr.append(td); }
  $('drones').append(tr); return tr;
});
function setNotice(message, kind='') { $('notice').textContent=message;$('notice').className='notice '+kind; }
async function command(action, values={}) {
  if (sending) return;
  sending=true;localError='';if(current) render(current);
  try {
    const response=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json','X-Panel-Token':token},body:JSON.stringify({action,...values}),signal:AbortSignal.timeout(5000)});
    const result=await response.json();
    if(!response.ok) throw new Error(result.error || 'Команда не выполнена');
  } catch(error) {localError=error.message;}
  finally {sending=false;}
}
$('prepare').onclick=()=>command('prepare',{record:$('record').checked});
$('hold').onclick=()=>command('hold');
$('land').onclick=()=>command('land');
$('go-to-target').onclick=()=>target&&command('goal',{x:target.x,y:target.y});
$('clear-target').onclick=()=>{target=null;$('target-label').textContent='Точка не выбрана';$('clear-target').disabled=true;$('go-to-target').disabled=true;draw();};
document.querySelectorAll('[data-scenario]').forEach(button=>button.onclick=()=>command(button.dataset.scenario,{record:$('record').checked}));
$('formation-form').onsubmit=event=>{event.preventDefault();if(!$('apply').disabled) command('formation',{formation:$('formation').value,scale:Number($('scale').value),duration:Number($('duration').value)});};
function render(state) {
  current=state;
  const flight=state.flight,active=['starting','running','ready','landing'].includes(flight.phase);
  const available=state.connected&&!active&&!sending&&state.action.state!=='pending'&&state.drones.every(d=>!d.armed);
  const ready=state.connected&&flight.phase==='ready'&&flight.kind==='operator'&&!sending&&state.action.state!=='pending';
  $('connection').textContent=state.connected?'Четыре аппарата подключены':'Ожидание свежих данных';
  $('connection').className='badge'+(state.connected?' connected':'');
  $('prepare').disabled=!available;$('hold').disabled=!(state.connected&&flight.phase==='ready'&&flight.kind==='operator')||sending;
  $('land').disabled=!(active||state.drones.some(d=>d.armed))||flight.phase==='landing'||sending;
  $('apply').disabled=!ready||!state.formation_settled;
  $('go-to-target').disabled=!target||!ready||sending;
  $('clear-target').disabled=!target;
  document.querySelectorAll('[data-scenario]').forEach(button=>button.disabled=!available);
  $('record').disabled=active;
  $('flight-status').textContent=phases[flight.phase]||'Ожидание';
  $('sim-time').textContent=`Время ${state.time.toFixed(1)} с`;
  if(localError) setNotice(localError,'error');
  else if(flight.phase==='error') setNotice('Полёт завершился с ошибкой. Проверьте события и подтверждение посадки.','error');
  else if(flight.phase==='landing') setNotice('Выполняется посадка. Дождитесь разоружения всех аппаратов.','pending');
  else if(['starting','running'].includes(flight.phase)) setNotice('Выполняется подготовка или сценарий. Посадка доступна в любой момент.','pending');
  else if(['complete','cancelled'].includes(flight.phase)) setNotice('Посадка подтверждена. Можно начать новый полёт.');
  else if(flight.phase==='ready'&&state.action.state==='ok'&&state.action.message.startsWith('Подготовка')) setNotice('Группа готова. Выберите строй или завершите полёт.');
  else setNotice(state.action.message,state.action.state==='error'?'error':state.action.state==='pending'?'pending':'');
  state.drones.forEach((drone,i)=>{
    const cells=rows[i].children;
    cells[1].textContent=!drone.fresh?'Нет свежих данных':!drone.armed?'На земле':drone.mode==='18'?'Посадка':drone.position[2]<.7?'Подготовка':'В полёте';
    cells[1].className=drone.fresh?'fresh':'stale';
    cells[2].textContent=drone.fresh?`${drone.position[2].toFixed(2)} м`:'—';
    cells[3].textContent=drone.fresh?`${Math.hypot(...drone.velocity).toFixed(2)} м/с`:'—';
  });
  const formation=state.formation;
  const formationLabel=formation?.mode==='FOLLOWING'&&!state.formation_settled?'Занимают места':modes[formation?.mode];
  $('formation-status').textContent=formation?`${shapes[formation.formation]||''} · ${formationLabel||'Ожидание'}`:'Нет свежих данных';
  $('errors').textContent=formation&&Array.isArray(formation.errors)&&formation.errors.length===3?`Ошибки по LiDAR: ${formation.errors.map(e=>Number(e).toFixed(2)).join(' / ')} м`:'Ошибки строя: —';
  const translate=line=>names.reduce((text,name,i)=>text.replaceAll(`uav_${i}`,name),line).replaceAll('column','колонна').replaceAll('wedge','клин').replaceAll('line','линия');
  const text=flight.logs.slice(-24).map(translate).join('\n')||'Событий пока нет.';
  if($('events').textContent!==text){$('events').textContent=text;$('events').scrollTop=$('events').scrollHeight;}
  $('output').textContent=flight.output?`Запись: ${flight.output.split('/').slice(-3).join('/')}`:'';
  draw();
}
function draw() {
  const canvas=$('map'),rect=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1;
  canvas.width=Math.round(rect.width*ratio);canvas.height=Math.round(rect.height*ratio);
  const ctx=canvas.getContext('2d');ctx.scale(ratio,ratio);
  const w=rect.width,h=rect.height,valid=(current?.drones||[]).filter(d=>d.fresh);
  if(!valid.length){ctx.fillStyle='#9babb9';ctx.font='13px system-ui';ctx.textAlign='center';ctx.fillText('Положение появится после подключения',w/2,h/2);return;}
  if(!view.ready){const xs=valid.map(d=>d.position[0]).concat(groundMarks.map(mark=>mark.x)),ys=valid.map(d=>d.position[1]).concat(groundMarks.map(mark=>mark.y));view.cx=(Math.min(...xs)+Math.max(...xs))/2;view.cy=(Math.min(...ys)+Math.max(...ys))/2;view.scale=Math.min((w-90)/Math.max(16,Math.max(...xs)-Math.min(...xs)),(h-80)/Math.max(16,Math.max(...ys)-Math.min(...ys)));view.ready=true;}
  const cx=view.cx,cy=view.cy,scale=view.scale;
  const px=x=>w/2+(x-cx)*scale,py=y=>h/2-(y-cy)*scale;
  ctx.lineWidth=1;ctx.strokeStyle='#27343f';ctx.fillStyle='#8295a6';ctx.font='10px system-ui';ctx.textAlign='center';
  const step=scale<8?10:2;
  for(let x=Math.floor((cx-w/scale/2)/step)*step;x<cx+w/scale/2;x+=step){ctx.beginPath();ctx.moveTo(px(x),0);ctx.lineTo(px(x),h);ctx.stroke();ctx.fillText(x.toFixed(0),px(x),h-9);}
  for(let y=Math.floor((cy-h/scale/2)/step)*step;y<cy+h/scale/2;y+=step){ctx.beginPath();ctx.moveTo(0,py(y));ctx.lineTo(w,py(y));ctx.stroke();}
  // Цвета и координаты совпадают с разметкой training_ground в Gazebo.
  ctx.lineWidth=2;
  ctx.strokeStyle='#ef604f';ctx.beginPath();ctx.moveTo(px(-100),py(0));ctx.lineTo(px(100),py(0));ctx.stroke();
  ctx.strokeStyle='#4f86ed';ctx.beginPath();ctx.moveTo(px(0),py(-100));ctx.lineTo(px(0),py(100));ctx.stroke();
  ctx.font='10px system-ui';ctx.textAlign='left';ctx.fillStyle='#ef8a7d';ctx.fillText('X',Math.min(w-14,px(100)-8),Math.max(12,py(0)-6));
  ctx.fillStyle='#82aaf4';ctx.fillText('Y',Math.min(w-14,px(0)+6),Math.max(12,py(100)+12));
  groundMarks.forEach(mark=>{const x=px(mark.x),y=py(mark.y);ctx.fillStyle=mark.color;ctx.strokeStyle='#10151b';ctx.lineWidth=1.5;ctx.beginPath();ctx.arc(x,y,5.5,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.font='10px system-ui';ctx.textAlign='center';ctx.fillText(mark.label,x,y-9);});
  const occupied=valid.map(drone=>({x:px(drone.position[0])-8,y:py(drone.position[1])-8,w:16,h:16}));
  valid.forEach(drone=>{
    ctx.lineWidth=1.5;
    const x=px(drone.position[0]),y=py(drone.position[1]);ctx.fillStyle=drone.id===0?'#f09588':'#cde8dc';ctx.beginPath();ctx.arc(x,y,drone.id===0?6:4.5,0,Math.PI*2);ctx.fill();
    ctx.strokeStyle=ctx.fillStyle;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+14*Math.cos(drone.yaw),y-14*Math.sin(drone.yaw));ctx.stroke();
    ctx.font='11px system-ui';ctx.textAlign='left';
    const label=names[drone.id],width=ctx.measureText(label).width;
    let chosen=null;
    for(let ring=0;ring<5&&!chosen;ring++){
      const distance=12+ring*16;
      for(const [lx,ly] of [[x+distance,y-distance],[x+distance,y+distance+8],[x-width-distance,y-distance],[x-width-distance,y+distance+8]]){
        const candidate={x:lx-3,y:ly-12,w:width+6,h:17};
        if(candidate.x<2||candidate.y<2||candidate.x+candidate.w>w-2||candidate.y+candidate.h>h-2)continue;
        if(!occupied.some(other=>candidate.x<other.x+other.w&&candidate.x+candidate.w>other.x&&candidate.y<other.y+other.h&&candidate.y+candidate.h>other.y)){chosen={...candidate,lx,ly};break;}
      }
    }
    if(chosen){
      occupied.push(chosen);ctx.strokeStyle=ctx.fillStyle;ctx.lineWidth=.7;
      ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(Math.max(chosen.x,Math.min(x,chosen.x+chosen.w)),Math.max(chosen.y,Math.min(y,chosen.y+chosen.h)));ctx.stroke();
      ctx.strokeStyle='#121a22';ctx.lineWidth=3;ctx.strokeText(label,chosen.lx,chosen.ly);ctx.fillText(label,chosen.lx,chosen.ly);
    }
  });
  if(target){const x=px(target.x),y=py(target.y);ctx.strokeStyle='#e9bf77';ctx.lineWidth=2;ctx.beginPath();ctx.arc(x,y,8,0,Math.PI*2);ctx.stroke();ctx.beginPath();ctx.moveTo(x-12,y);ctx.lineTo(x+12,y);ctx.moveTo(x,y-12);ctx.lineTo(x,y+12);ctx.stroke();}
}
const map=$('map');
map.addEventListener('wheel',event=>{event.preventDefault();const rect=map.getBoundingClientRect(),mx=event.clientX-rect.left,my=event.clientY-rect.top;const before=[(mx-rect.width/2)/view.scale+view.cx,(rect.height/2-my)/view.scale+view.cy];view.scale=Math.max(3,Math.min(80,view.scale*Math.exp(-event.deltaY*.001)));view.cx=before[0]-(mx-rect.width/2)/view.scale;view.cy=before[1]+(my-rect.height/2)/view.scale;view.ready=true;draw();},{passive:false});
map.addEventListener('pointerdown',event=>{map.setPointerCapture(event.pointerId);dragging={x:event.clientX,y:event.clientY,cx:view.cx,cy:view.cy};});
map.addEventListener('pointercancel',()=>{dragging=null;});
map.addEventListener('lostpointercapture',()=>{dragging=null;});
map.addEventListener('pointermove',event=>{if(!dragging)return;const rect=map.getBoundingClientRect();view.cx=dragging.cx-(event.clientX-dragging.x)/view.scale;view.cy=dragging.cy+(event.clientY-dragging.y)/view.scale;draw();});
map.addEventListener('pointerup',event=>{if(!dragging)return;const moved=Math.hypot(event.clientX-dragging.x,event.clientY-dragging.y);if(moved<5){const rect=map.getBoundingClientRect();target={x:view.cx+(event.clientX-rect.left-rect.width/2)/view.scale,y:view.cy-(event.clientY-rect.top-rect.height/2)/view.scale};$('target-label').textContent=`Цель: X=${target.x.toFixed(1)} м, Y=${target.y.toFixed(1)} м`;}$('clear-target').disabled=!target;dragging=null;draw();});
new ResizeObserver(draw).observe($('map'));
async function poll(){
  try{const response=await fetch('/api/state',{signal:AbortSignal.timeout(3000)});if(!response.ok)throw new Error();render(await response.json());}
  catch{setNotice('Панель недоступна. Проверьте её терминал; закрытие страницы не завершает полёт.','error');$('connection').textContent='Нет связи с панелью';$('connection').className='badge';document.querySelectorAll('button').forEach(b=>b.disabled=true);current=null;draw();}
  setTimeout(poll,500);
}
poll();
